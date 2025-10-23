#![allow(dead_code)]
#![allow(unused)]
use adxl345_driver2::{Adxl345, Adxl345Reader, Adxl345Writer, i2c::Device};
use anyhow::{Result, *};
use rppal::gpio::{Gpio, InputPin, Pin};
use rppal::{i2c::I2c, system::DeviceInfo};
use std::result::Result::Ok;
use std::{
    sync::mpsc::{self, Receiver, Sender, channel},
    thread::{self, *},
    time::{self, Duration},
};

fn main() -> Result<()> {
    // let (mut i2c, mut device, mut pin) = init(0, 23)?;
    let gpio = Gpio::new()?;
    let mut pin = gpio.get(23)?.into_input_pullup();

    loop {
        sleep(Duration::from_millis(200));
        // if let Ok((x, y, z)) = device.acceleration() {
        //     println!("X: {}, Y: {}, Z: {}", x, y, z);
        // }
        if pin.is_low() {
            println!("Pin is LOW");
        } else {
            println!("Pin is HIGH");
        }
    }
}

/// Main loop for the accelerometer and its tire
fn tire(tx: Sender<bool>, tire_bus: u8, pin_num: u8) -> Result<()> {
    let (mut i2c, mut adxl345, mut pin) = init(tire_bus, pin_num)?;
    let mut current_speed = 1.0;

    let (transmitter, reciever) = channel::<f32>();

    loop {
        let timer = time::Instant::now();

        if let Ok(speed) = reciever.try_recv() {
            current_speed = speed;
        }

        if pin.is_high() {
            current_speed = frequency_to_speed(1.0 / timer.elapsed().as_secs_f32())
        }

        // Update frequency based on accelerometer readings
        let has_grip = grip(adxl345.acceleration()?, current_speed);

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
            // Magnet detected
            transmitter.send(0.0).unwrap();
        } else {
            transmitter.send(0.0).unwrap();
        }
        transmitter.send(frequency_to_speed(frequency_to_speed(
            1.0 / timer.elapsed().as_secs_f32(),
        )));
    }
}

/// Convert frequency in Hz to speed in m/s
fn frequency_to_speed(frequency: f32) -> f32 {
    return (frequency / 16.0) * std::f32::consts::PI * 0.7; // frequency / 16 to get
    // rotations per second, then multiply by circumference (pi * diameter) d = 0.7m to get the
    // speed in m/s
}

/// Determine if grip is good based on wheel acceleration and bike acceleration
fn grip(acceleration: (i16, i16, i16), speed: f32) -> bool {
    return false;
}

/// Returns readings from device
fn get_readings(adxl345: &mut Device<I2c>) -> Result<(f32, f32, f32)> {
    println!(
        "I2C example started on a {}",
        DeviceInfo::new()
            .context("Failed to get new DeviceInfo")?
            .model()
    );
    let f = adxl345.acceleration()?;
    Ok((0.0, 0.0, 0.0))
}

/// generates i2c device and accelerometer
fn init(bus: u8, pin: u8) -> Result<(I2c, Device<I2c>, InputPin)> {
    let i2c = I2c::with_bus(bus)?;
    let mut adxl345 = Device::new(I2c::with_bus(bus)?).context("failed").unwrap();

    let gpio = Gpio::new()?;
    let mut pin = gpio.get(23)?.into_input_pullup();

    return Ok((i2c, adxl345, pin));
}
