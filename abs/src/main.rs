#![allow(dead_code)]
#![allow(unused)]
use adxl345_driver2::{Adxl345, Adxl345Reader, Adxl345Writer, i2c::Device};
use anyhow::{Result, *};
use rppal::gpio::{Gpio, InputPin, Pin};
use rppal::{i2c::I2c, system::DeviceInfo};
use std::collections::VecDeque;
use std::result::Result::Ok;
use std::{
    sync::mpsc::{self, Receiver, Sender, channel},
    thread::{self, *},
    time::{self, Duration},
};

#[derive(Debug)]
struct GripState {
    estimated_pitch: f32,
    estimated_roll: f32,
    last_update_time: time::Instant,
}

impl GripState {
    fn new() -> Self {
        Self {
            estimated_pitch: 0.0,
            estimated_roll: 0.0,
            last_update_time: time::Instant::now(),
        }
    }
}

fn main() -> Result<()> {
    println!("Hello, world!");

    let mut grip_status = (true, true);

    let (emitter1, reciever1) = channel::<bool>();
    let (emitter2, reciever2) = channel::<bool>();

    let tire1_handle = thread::spawn(move || tire(emitter1, 0, 20));
    let tire2_handle = thread::spawn(move || tire(emitter2, 1, 21));

    loop {
        if let Ok(msg) = reciever1.try_recv() {
            grip_status.0 = msg;
        }

        if let Ok(msg) = reciever2.try_recv() {
            grip_status.1 = msg;
        }

        println!("status: {:?}", grip_status);
    }

    tire1_handle.join().unwrap();
    tire2_handle.join().unwrap();
}

/// Main loop for the accelerometer and its tire
fn tire(tx: Sender<bool>, tire_bus: u8, pin_num: u8) -> Result<()> {
    let (mut i2c, mut adxl345, mut pin) = init(tire_bus, pin_num)?;
    let pin = pin.into_input_pullup();
    let mut current_speed = 1.0;

    let (transmitter, reciever) = channel::<f32>();
    let magnet_handle = thread::spawn(move || magnets(pin, transmitter));

    let mut speeds: VecDeque<(f32, f32)> = VecDeque::new();
    let mut grip_state = GripState::new();

    let mut timer = time::Instant::now();

    loop {
        if let Ok(mag_speed) = reciever.try_recv() {
            speeds.push_back((mag_speed, timer.elapsed().as_secs_f32()));
            if speeds.len() > 50 {
                speeds.pop_front();
            }
        }

        // Update frequency based on accelerometer readings
        let has_grip = grip(adxl345.acceleration()?, &speeds, &mut grip_state);

        // Send frequency update to main thread
        let msg = has_grip;

        println!("grip state: {:?}", &grip_state);
        // If send fails, the receiver has been dropped, so exit
        if tx.send(msg).is_err() {
            break;
        }

        timer = time::Instant::now();
    }

    Ok(())
}

fn magnets(pin: InputPin, transmitter: Sender<f32>) -> Result<()> {
    let mut activated = false;

    let mut timer = time::Instant::now();

    let mut queue: VecDeque<time::Instant> = VecDeque::new();

    loop {
        if pin.is_low() {
            // Magnet detected, idk why low means detected
            queue.push_back(time::Instant::now());

            transmitter.send({
                // calculate frequency by averaging last few intervals
                if queue.len() < 2 {
                    0.0
                } else {
                    while queue.len() > 5 {
                        queue.pop_front();
                    }

                    let mut intervals = Vec::new();
                    for i in 1..queue.len() {
                        let dt = queue[i].duration_since(queue[i - 1]).as_secs_f32();
                        intervals.push(dt);
                    }
                    let avg_interval = intervals.iter().sum::<f32>() / intervals.len() as f32;
                    frequency_to_speed(1.0 / avg_interval)
                }
            })?;

            timer = time::Instant::now();
        }
    }
}

/// Convert frequency in Hz to speed in m/s
fn frequency_to_speed(frequency: f32) -> f32 {
    return (frequency / 27.0) * std::f32::consts::PI * 0.7; // frequency / 27 (magnets) to get
    // rotations per second, then multiply by circumference (pi * diameter) d = 0.7m to get the
    // speed in m/s
}

fn grip(
    acceleration: (i16, i16, i16),
    speeds: &VecDeque<(f32, f32)>,
    state: &mut GripState,
) -> bool {
    const ACCEL_SCALE: f32 = 2.0 / 512.0;
    const GRAVITY: f32 = 9.81;
    const CONSISTENCY_THRESHOLD: f32 = 3.0; // variance threshold (m/s^2)
    const MIN_SPEED: f32 = 0.5;
    const WINDOW_SIZE: usize = 5;
    const SLIP_THRESHOLD: f32 = 0.30; // % deviation threshold
    const MAX_GRIP_TOTAL: f32 = 0.8; // Maximum grip coefficient for bike tires on dry road: mu = F_tire / (mass * g)
    const GRAVITY_FILTER_TAU: f32 = 0.5; // Time constant for low-pass filter (seconds) -- 
    // smaller = faster response, more noise | larger = slower response, less noise

    if speeds.len() < WINDOW_SIZE {
        return true;
    }

    // wheel acceleration calculations
    let mut wheel_accels = Vec::new();
    for i in 0..(WINDOW_SIZE - 1) {
        let (speed_new, _) = speeds[i];
        let (speed_old, dt) = speeds[i + 1];
        if dt > 0.0 {
            wheel_accels.push((speed_new - speed_old) / dt);
        }
    }

    if wheel_accels.is_empty() {
        return true;
    }

    let wheel_mean_accel = wheel_accels.iter().sum::<f32>() / wheel_accels.len() as f32;
    let variance = wheel_accels
        .iter()
        .map(|a| (a - wheel_mean_accel).powi(2))
        .sum::<f32>()
        / wheel_accels.len() as f32;

    let ax = acceleration.0 as f32 * ACCEL_SCALE; // forward/backward
    let ay = acceleration.1 as f32 * ACCEL_SCALE; // left/right 
    let az = acceleration.2 as f32 * ACCEL_SCALE; // up/down 

    let now = time::Instant::now();
    let dt = now.duration_since(state.last_update_time).as_secs_f32();
    state.last_update_time = now;

    // Calculate instantaneous pitch and roll angles
    let pitch_instant = (ax / az.max(0.1)).atan();
    let roll_instant = (ay / az.max(0.1)).atan();

    // Low-pass filter for pitch and roll
    let alpha = dt / (GRAVITY_FILTER_TAU + dt);

    // blend estimate with measurement
    state.estimated_pitch = state.estimated_pitch * (1.0 - alpha) + pitch_instant * alpha;
    state.estimated_roll = state.estimated_roll * (1.0 - alpha) + roll_instant * alpha;

    // use filtered values for gravity compensation
    let pitch = state.estimated_pitch;
    let roll = state.estimated_roll;

    // Gravity components in bike's reference frame (using filtered angles)
    let gravity_forward = GRAVITY * pitch.sin();
    let gravity_lateral = GRAVITY * roll.sin();

    // TRUE bike acceleration (motion) = measured - gravity
    let true_forward_accel = (ax * GRAVITY) - gravity_forward;
    let true_lateral_accel = (ay * GRAVITY) - gravity_lateral;

    //slippy dippy
    let variance_ok = variance < CONSISTENCY_THRESHOLD;

    //accelerometer vs wheel speed check
    let accel_deviation = (true_forward_accel - wheel_mean_accel).abs();
    let relative_deviation = if wheel_mean_accel.abs() > 1.0 {
        accel_deviation / wheel_mean_accel.abs()
    } else {
        accel_deviation / 2.0
    };
    let correlation_ok = relative_deviation < SLIP_THRESHOLD;

    // total grip check
    let lateral_grip_demand = true_lateral_accel.abs() / GRAVITY;
    let forward_grip_demand = true_forward_accel.abs() / GRAVITY;

    let total_grip_demand = (lateral_grip_demand.powi(2) + forward_grip_demand.powi(2)).sqrt();
    let grip_budget_ok = total_grip_demand < MAX_GRIP_TOTAL;

    // spike detection
    let instant_forward = (ax * GRAVITY) - (GRAVITY * pitch_instant.sin());
    let instant_lateral = (ay * GRAVITY) - (GRAVITY * roll_instant.sin());
    let sudden_spike = instant_forward.abs() > 8.0 || instant_lateral.abs() > 8.0;

    // THE FINALE
    variance_ok && correlation_ok && grip_budget_ok && !sudden_spike
}

/// Returns readings from device
fn get_readings(adxl345: &mut Device<I2c>) -> Result<(i16, i16, i16)> {
    println!(
        "I2C started on {}",
        DeviceInfo::new()
            .context("Failed to get new DeviceInfo")?
            .model()
    );
    let f = adxl345.acceleration()?;
    Ok(f)
}

/// generates i2c device and accelerometer
fn init(bus: u8, pin: u8) -> Result<(I2c, Device<I2c>, Pin)> {
    let i2c = I2c::with_bus(bus)?;
    let mut adxl345 = Device::new(I2c::with_bus(bus)?).context("failed").unwrap();

    let gpio = Gpio::new()?;
    let mut pin = gpio.get(pin)?;

    return Ok((i2c, adxl345, pin));
}
