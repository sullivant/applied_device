extern crate modbus;

use modbus::tcp;
use modbus::Client;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;
use yaml_rust::YamlLoader;

static ENCODER_POS_1_REG: u16 = 4;
static ENCODER_POS_2_REG: u16 = 5;
static MAX_REGISTER: u16 = 56; // The last register we really care about seeing
static MAX_32_BIT: u64 = 65536;

pub struct AppliedDevice {
    servo_name: String,    // The provided name of this applied servo
    servo_address: String, // The IP/Hostname of the device
    client: tcp::Transport,
    resource_location: String, // the location of the configuration file for this device
}

impl AppliedDevice {
    pub fn get_encoder_count(&mut self) -> u64 {
        //let mut client = tcp::Transport::new("192.168.0.200").unwrap();

        let x: u16 = *self
            .client
            .read_holding_registers(ENCODER_POS_1_REG, 1)
            .expect("IO Error")
            .first()
            .unwrap();

        let y: u16 = *self
            .client
            .read_holding_registers(ENCODER_POS_2_REG, 1)
            .expect("IO Error")
            .first()
            .unwrap();

        let encoder_position: u64 = y as u64 + (x as u64 * MAX_32_BIT);

        encoder_position
    }

    pub fn dump_registers(&mut self) {
        println!("Dumping registers up to {}", MAX_REGISTER);
        for (n, i) in self
            .client
            .read_holding_registers(0, MAX_REGISTER)
            .expect("IO Error")
            .iter()
            .enumerate()
        {
            println!("Register {}: {}", n, i);
        }
        println!("Done reading registers.");
    }

    pub fn get_name(&mut self) -> &String {
        &self.servo_name
    }

    pub fn get_address(&mut self) -> &String {
        &self.servo_address
    }

    pub fn get_resource_location(&mut self) -> &String {
        &self.resource_location
    }

    pub fn to_string(&mut self) -> String {
        format!(
            "For applied device {} using address of: {}",
            &self.servo_name, &self.servo_address
        )
    }
}

pub fn new(device_name: String, servo_name: String) -> AppliedDevice {
    let resource_location: String = format!("./thingy/resources/{}.yaml", device_name);
    println!("Creating applied device: {}", device_name);
    println!("Using device configuration at: {}", resource_location);
    let file = File::open(resource_location.clone()).expect("Unable to open file.");
    let mut buf_reader = BufReader::new(file);
    let mut contents = String::new();
    buf_reader
        .read_to_string(&mut contents)
        .expect("Unable to read input file.");

    let device_yaml = YamlLoader::load_from_str(&contents).unwrap();
    let device_conf = &device_yaml[0];

    // Get device coupler information
    let coupler_raw = &device_conf["device"][servo_name.as_str()];
    let coupler = coupler_raw.as_str().unwrap_or("127.0.0.1");
    let client = tcp::Transport::new(coupler).unwrap();

    AppliedDevice {
        servo_name: servo_name.to_string(),
        servo_address: coupler.to_string(),
        client: client,
        resource_location: resource_location,
    }
}
