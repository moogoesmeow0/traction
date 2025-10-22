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

    loop {
        let timer = time::Instant::now();

        if let Ok(speed) = reciever.try_recv() {
            current_speed = speed;
        }

        if let Ok(mag_speed) = reciever.try_recv() {
            speeds.push_back((mag_speed, timer.elapsed().as_secs_f32()));
            if speeds.len() > 50 {
                speeds.pop_front();
            }
        }

        // Update frequency based on accelerometer readings
        let has_grip = grip(adxl345.acceleration()?, &speeds);

        // Send frequency update to main thread
        let msg = has_grip;

        // If send fails, the receiver has been dropped, so exit
        if tx.send(msg).is_err() {
            break;
        }
    }

    Ok(())
}

fn magnets(pin: InputPin, transmitter: Sender<f32>) -> Result<()> {
    let mut activated = false;

    let timer = time::Instant::now();
    sleep(Duration::from_millis(5));

    loop {
        if pin.is_low() {
            // Magnet detected, idk why low means detected
            transmitter.send(0.0).unwrap();
        } else {
            transmitter.send(0.0).unwrap();
        }
        transmitter.send((frequency_to_speed(1.0 / timer.elapsed().as_secs_f32())));
    }
}

/// Convert frequency in Hz to speed in m/s
fn frequency_to_speed(frequency: f32) -> f32 {
    return (frequency / 27.0) * std::f32::consts::PI * 0.7; // frequency / 27 (magnets) to get
    // rotations per second, then multiply by circumference (pi * diameter) d = 0.7m to get the
    // speed in m/s
}

/// Determine if grip is good based on wheel acceleration and bike acceleration
/// acceleration: (x, y, z) from accelerometer
/// speeds: VecDeque of (speed, delta_time) tuples, lower index is more recent
fn grip(acceleration: (i16, i16, i16), speeds: &VecDeque<(f32, f32)>) -> bool {
    const ACCEL_SCALE: f32 = 2.0 / 512.0;
    const GRAVITY: f32 = 9.81;
    const CONSISTENCY_THRESHOLD: f32 = 3.0; // m/s² variance threshold
    const MIN_SPEED: f32 = 0.5;
    const WINDOW_SIZE: usize = 5;

    if speeds.len() < WINDOW_SIZE {
        return true;
    }

    // Calculate wheel acceleration consistency over window
    let mut wheel_accels = Vec::new();
    for i in 0..WINDOW_SIZE - 1 {
        let (speed_new, _) = speeds[i];
        let (speed_old, dt) = speeds[i + 1];
        if dt > 0.0 {
            wheel_accels.push((speed_new - speed_old) / dt);
        }
    }

    if wheel_accels.is_empty() {
        return true;
    }

    // Calculate variance in wheel acceleration
    let mean = wheel_accels.iter().sum::<f32>() / wheel_accels.len() as f32;
    let variance =
        wheel_accels.iter().map(|a| (a - mean).powi(2)).sum::<f32>() / wheel_accels.len() as f32;

    // KEY INSIGHT: When grip is good, wheel acceleration is smooth and consistent
    // When slipping, wheel acceleration becomes erratic (variance increases)
    // This works on ANY slope because we're looking at CHANGES, not absolute values

    // Also check for sudden spikes in accelerometer (indicates wheel breaking free)
    let ax = acceleration.0 as f32 * ACCEL_SCALE * GRAVITY;
    let sudden_spike = ax.abs() > 7.0; // More than ~0.7g forward acceleration is suspicious

    !sudden_spike && variance < CONSISTENCY_THRESHOLD
}

/// Returns readings from device
fn get_readings(adxl345: &mut Device<I2c>) -> Result<(f32, f32, f32)> {
    println!(
        "I2C started on {}",
        DeviceInfo::new()
            .context("Failed to get new DeviceInfo")?
            .model()
    );
    let f = adxl345.acceleration()?;
    Ok((0.0, 0.0, 0.0))
}

/// generates i2c device and accelerometer
fn init(bus: u8, pin: u8) -> Result<(I2c, Device<I2c>, Pin)> {
    let i2c = I2c::with_bus(bus)?;
    let mut adxl345 = Device::new(I2c::with_bus(bus)?).context("failed").unwrap();

    let gpio = Gpio::new()?;
    let mut pin = gpio.get(23)?;

    return Ok((i2c, adxl345, pin));
}
