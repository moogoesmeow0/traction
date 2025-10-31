#![allow(dead_code)]
#![allow(unused)]
use adxl345_driver2::{Adxl345, Adxl345Reader, Adxl345Writer, i2c::Device};
use anyhow::{Result, *};
use log::{debug, error, info, log};
use rppal::gpio::{Gpio, InputPin, Level, Pin, Trigger};
use rppal::{i2c::I2c, system::DeviceInfo};
use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};
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

    TermLogger::init(
        LevelFilter::Info,
        Config::default(),
        TerminalMode::Mixed,
        ColorChoice::Auto,
    )?;

    error!("Bright red error");
    info!("This only appears in the log file");
    debug!("This level is currently not enabled for any logger");
    info!(
        "pi model: {:?}",
        DeviceInfo::new()
            .or_else(|e| {
                error!("Error getting device info: {:?}", e);
                Err(e)
            })?
            .model()
    );

    let mut grip_status = (true, true);

    let (emitter1, reciever1) = channel::<bool>();
    let (emitter2, reciever2) = channel::<bool>();

    let tire1_handle = thread::spawn(move || tire(emitter1, 0, 23, 0x53));
    let tire2_handle = thread::spawn(move || tire(emitter2, 1, 24, 0x1D));

    loop {
        sleep(Duration::from_millis(200));
        if let Ok(msg) = reciever1.try_recv() {
            grip_status.0 = msg;
        }

        if let Ok(msg) = reciever2.try_recv() {
            grip_status.1 = msg;
        }

        info!("status: {:?}", grip_status);
    }

    tire1_handle.join().unwrap();
    tire2_handle.join().unwrap();
}

/// Main loop for the accelerometer and its tire
fn tire(tx: Sender<bool>, tire_bus: u8, pin_num: u8, id: u8) -> Result<()> {
    let (mut adxl345, mut pin) = init(tire_bus, id, pin_num)?;
    let pin = pin.into_input_pullup();
    let mut current_speed = 1.0;

    let (transmitter, reciever) = channel::<f32>();
    let magnet_handle = thread::spawn(move || magnets(pin, transmitter));

    let mut speeds: VecDeque<(f32, time::Instant)> = VecDeque::new();
    let mut grip_state = GripState::new();

    let mut timer = time::Instant::now();

    loop {
        timer = time::Instant::now();

        if let Ok(mag_speed) = reciever.try_recv() {
            speeds.push_back((mag_speed, timer));
            if speeds.len() > 50 {
                speeds.pop_front();
            }
        }

        // Update frequency based on accelerometer readings
        let has_grip = grip(get_readings(&mut adxl345)?, &speeds, &mut grip_state);

        // Send frequency update to main thread
        let msg = has_grip;

        info!("grip state: {:?}", &grip_state);
        // If send fails, the receiver has been dropped, so exit
        if tx.send(msg).is_err() {
            break;
        }
    }

    Ok(())
}

fn magnets(mut pin: InputPin, transmitter: Sender<f32>) -> Result<()> {
    let mut activated = false;

    let mut timer = time::Instant::now();

    let mut queue: VecDeque<time::Instant> = VecDeque::new();

    let (tx, rx) = mpsc::channel();

    pin.set_async_interrupt(
        Trigger::FallingEdge,
        Some(Duration::from_millis(10)),
        move |_level| {
            let _ = tx.send(());
        },
    );

    loop {
        match rx.recv_timeout({
            if queue.len() > 1 {
                queue[0].duration_since(queue[1]).mul_f32(1.1)
            } else {
                Duration::from_millis(200)
            }
        }) {
            Ok(_) => {
                timer = time::Instant::now();

                // Magnet detected, idk why low means detected
                queue.push_front(time::Instant::now());

                info!("Averaged out speed: {:?}", {
                    // calculate frequency by averaging last few intervals
                    if queue.len() < 2 {
                        0.0
                    } else {
                        while queue.len() > 5 {
                            queue.pop_back();
                        }

                        let mut intervals = Vec::new();
                        for i in 0..(queue.len() - 1) {
                            let dt = queue[i].duration_since(queue[i + 1]).as_secs_f32();
                            intervals.push(dt);
                        }
                        let avg_interval = intervals.iter().sum::<f32>() / intervals.len() as f32;
                        frequency_to_speed(1.0 / avg_interval)
                    }
                });

                transmitter.send({
                    frequency_to_speed(1.0 / queue[0].duration_since(queue[1]).as_secs_f32())
                })?;
            }
            Err(_) => {
                transmitter.send(frequency_to_speed(1.0 / timer.elapsed().as_secs_f32()))?;
            }
        }
    }
}

/// Convert frequency in Hz to speed in m/s
fn frequency_to_speed(frequency: f32) -> f32 {
    return (frequency / 27.0) * std::f32::consts::PI * 0.7; // frequency / 27 (magnets) to get
    // rotations per second, then multiply by circumference (pi * diameter) d = 0.7m to get the
    // speed in m/s
}

/// Determines if the tire has grip based on accelerometer and wheel speed data
/// # Arguments
/// * `acceleration` - Tuple of (ax, ay, az) accelerations in m/s^2
fn grip(
    acceleration: (f32, f32, f32),
    speeds: &VecDeque<(f32, time::Instant)>, // Changed to store Instant
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
    for i in 0..(speeds.len() - 1) {
        let (speed_new, time_new) = speeds[i];
        let (speed_old, time_old) = speeds[i + 1];
        let dt = time_new.duration_since(time_old).as_secs_f32();
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

    let ax = acceleration.0; // backward/forward
    let ay = acceleration.1; // left/right 
    let az = acceleration.2; // up/down 

    let now = time::Instant::now();
    let dt = now.duration_since(state.last_update_time).as_secs_f32();
    state.last_update_time = now;

    // --- CORRECTED MATH ---
    // Calculate instantaneous pitch and roll angles based on the specified coordinate system.
    // Pitch: Rotation around Y-axis. Positive pitch = front of bike is tilted UP.
    // Roll: Rotation around X-axis. Positive roll = bike is tilted to the RIGHT.
    let pitch_instant = ax.atan2((ay.powi(2) + az.powi(2)).sqrt());
    let roll_instant = ay.atan2(az);

    // Low-pass filter for pitch and roll (this part is correct)
    let alpha = dt / (GRAVITY_FILTER_TAU + dt);

    // blend estimate with measurement
    state.estimated_pitch = state.estimated_pitch * (1.0 - alpha) + pitch_instant * alpha;
    state.estimated_roll = state.estimated_roll * (1.0 - alpha) + roll_instant * alpha;

    // use filtered values for gravity compensation
    let pitch = state.estimated_pitch;
    let roll = state.estimated_roll;

    // Gravity components in bike's reference frame (using filtered angles)
    // These formulas are derived from rotating the gravity vector (-g in Z) into the bike's frame.
    let gravity_backward = GRAVITY * pitch.sin(); // Component of gravity along the +X (backward) axis
    let gravity_lateral = -GRAVITY * pitch.cos() * roll.sin(); // Component of gravity along the +Y (right) axis

    // TRUE bike acceleration (motion) = measured - gravity
    // Note: We negate wheel_mean_accel because it's a forward acceleration, but our axis `ax` is backward.
    let true_backward_accel = ax - gravity_backward;
    let true_lateral_accel = ay - gravity_lateral;
    let true_forward_accel = -true_backward_accel;

    //slippy dippy
    let variance_ok = variance < CONSISTENCY_THRESHOLD;

    //accelerometer vs wheel speed check
    let accel_deviation = (true_forward_accel - wheel_mean_accel).abs();
    let relative_deviation = if wheel_mean_accel.abs() > 1.0 {
        accel_deviation / wheel_mean_accel.abs()
    } else {
        accel_deviation
    };
    let correlation_ok = relative_deviation < SLIP_THRESHOLD;

    // total grip check
    // We use the magnitude of true acceleration, so signs don't matter here.
    let lateral_grip_demand = true_lateral_accel.abs() / GRAVITY;
    let forward_grip_demand = true_forward_accel.abs() / GRAVITY;

    let total_grip_demand = (lateral_grip_demand.powi(2) + forward_grip_demand.powi(2)).sqrt();
    let grip_budget_ok = total_grip_demand < MAX_GRIP_TOTAL;

    // spike detection (using instantaneous angles for responsiveness)
    let instant_gravity_backward = GRAVITY * pitch_instant.sin();
    let instant_gravity_lateral = -GRAVITY * pitch_instant.cos() * roll_instant.sin();
    
    let instant_forward = -(ax - instant_gravity_backward);
    let instant_lateral = ay - instant_gravity_lateral;
    let sudden_spike = instant_forward.abs() > 8.0 || instant_lateral.abs() > 8.0;

    // THE FINALE
    variance_ok && correlation_ok && grip_budget_ok && !sudden_spike
}
/// Average Earth gravity in m/s²
const EARTH_GRAVITY_MS2: f32 = 9.80665;
const ACCEL_RAW_TO_G: f32 = 256.0;

/// Returns readings from device
/// returns (ax, ay, az) in m/s²
fn get_readings(adxl345: &mut Device<I2c>) -> Result<(f32, f32, f32)> {
    let (x_raw, y_raw, z_raw) = adxl345
        .acceleration()
        .context("Failed to get acceleration data")?;

    // Convert raw sensor values to g
    let x_g = x_raw as f32 / ACCEL_RAW_TO_G;
    let y_g = y_raw as f32 / ACCEL_RAW_TO_G;
    let z_g = z_raw as f32 / ACCEL_RAW_TO_G;

    info!("acceleration in g: {:?}", (x_g, y_g, z_g));

    let x = x_g * EARTH_GRAVITY_MS2;
    let y = y_g * EARTH_GRAVITY_MS2;
    let z = z_g * EARTH_GRAVITY_MS2;
    Ok((x, y, z))
}

/// generates i2c device and accelerometer
fn init(bus: u8, id: u8, pin: u8) -> Result<(Device<I2c>, Pin)> {
    let mut adxl345 = Device::with_address(I2c::new()?, id)?;

    // Set full scale output and range to 2G.
    adxl345
        .set_data_format(8)
        .context("Failed to set data format")?;
    // Set measurement mode on.
    adxl345
        .set_power_control(8)
        .context("Failed to turn on measurement mode")?;

    let id = adxl345.device_id().context("Failed to get device id")?;
    println!("Device id = {}", id);

    let gpio = Gpio::new()?;
    let mut pin = gpio.get(pin)?;
    Ok((adxl345, pin))
}
