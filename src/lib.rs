extern crate modbus;

use modbus::tcp;
use modbus::Client;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;
use std::time;
use std::time::Instant;
use yaml_rust::YamlLoader;

static ALARM_REG: u16 = 0;
static STATUS_REG: u16 = 1;
static ENCODER_POS_1_REG: u16 = 4;
static ENCODER_POS_2_REG: u16 = 5;
static MAX_REGISTER: u16 = 56; // The last register we really care about seeing
static MAX_32_BIT: u64 = 65536;
static MAX_HOMING_TIME: u64 = 60; // Max allowed time to home servo, in seconds
static MAX_MOVE_TIME: u64 = 60; // Max allowed time to execute a move, in seconds
static ENCODER_POSITION_RANGE: u64 = 1000; // Allowed +/- range value an encoder position

static ACCELERATION: u16 = 27;
static DECELERATION: u16 = 28;
static VELOCITY: u16 = 29;
static DISTANCE_1: u16 = 30;
static DISTANCE_2: u16 = 31;
static EXECUTE_COMMAND: u16 = 124;

// STATUS NAMES
pub static MOTOR_ENABLED: &str = "Motor Enabled";
pub static TUNING: &str = "Tuning";
pub static FAULT: &str = "Fault";
pub static IN_POSITION: &str = "In Position";
pub static MOVING: &str = "Moving";
pub static JOGGING: &str = "Jogging";
pub static STOPPING: &str = "Stopping";
pub static WAIT_FOR_INPUT: &str = "Wait for Input";
pub static SAVING: &str = "Saving";
pub static ALARM: &str = "Alarm";
pub static HOMING: &str = "Homing";
pub static DELAY: &str = "Delay";
pub static WIZARD_RUNNING: &str = "Wizard Running";
pub static INITIALIZING: &str = "Initializing";

static ALARM_CODE_NAMES: &[&str] = &[
    "Position Limit Error",
    "CCW Limit Error",
    "CW Limit Error",
    "Over Temp Error",
    "Internal Voltage Error",
    "Over Voltage Error",
    "Under Voltage Error",
    "Over Current Error",
    "Open Motor Winding Error",
    "Bad Encoder Error",
    "Comm Error",
    "Bad Flash Error",
    "No Move Error",
    "Motor resistance out of range",
    "Blank Q Segment",
    "No Move",
];

static STATUS_CODE_NAMES: &[&str] = &[
    MOTOR_ENABLED,
    TUNING,
    FAULT,
    IN_POSITION,
    MOVING,
    JOGGING,
    STOPPING,
    WAIT_FOR_INPUT,
    SAVING,
    ALARM,
    HOMING,
    DELAY,
    WIZARD_RUNNING,
    INITIALIZING,
];

pub struct AppliedDevice {
    servo_name: String,    // The provided name of this applied servo
    servo_address: String, // The IP/Hostname of the device
    client: tcp::Transport,
    resource_location: String, // the location of the configuration file for this device
    servo_status: Vec<String>,
    servo_alarm: Vec<String>,
}

impl AppliedDevice {
    pub fn get_encoder_count(&mut self) -> u64 {
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

    pub fn get_servo_alarms(&mut self) -> &Vec<String> {
        let read: usize = self.get_register_value(ALARM_REG) as usize;
        // Reset the current array of servo alarm values
        self.servo_alarm = Vec::new();

        for i in 0..ALARM_CODE_NAMES.len() {
            if read & (1 << i) != 0 {
                self.servo_status.push(ALARM_CODE_NAMES[i].to_string());
                // println!("{:16b} & {:16b} = {}", read, (1 << i), ALARM_CODE_NAMES[i]);
            }
        }

        &self.servo_alarm
    }

    pub fn get_servo_status(&mut self) -> &Vec<String> {
        let read: usize = self.get_register_value(STATUS_REG) as usize;
        // Reset the current array of servo status values
        self.servo_status = Vec::new();

        for i in 0..STATUS_CODE_NAMES.len() {
            if read & (1 << i) != 0 {
                self.servo_status.push(STATUS_CODE_NAMES[i].to_string());
                // println!("{:16b} & {:16b} = {}", read, (1 << i), STATUS_CODE_NAMES[i]);
            }
        }

        &self.servo_status
    }

    pub fn reset_alarm_or_fault(&mut self) {
        let mut alarm_present: bool = self.get_servo_status().contains(&ALARM.to_string());
        let mut fault_present: bool = self.get_servo_status().contains(&FAULT.to_string());
        let mut try_count: i8 = 0;

        if !alarm_present && !fault_present {
            self.enable_motor();
            return;
        }

        while alarm_present || fault_present {
            println!(
                "Found alarm: {} or fault: {}, trying to reset",
                alarm_present, fault_present
            );
            self.write_register(EXECUTE_COMMAND, 186);
            std::thread::sleep(time::Duration::from_millis(1000));

            if try_count > 2 {
                println!("!!Unable to reset alarm or fault!!");
                return;
            }
            try_count = try_count + 1;
            alarm_present = self.get_servo_status().contains(&ALARM.to_string());
            fault_present = self.get_servo_status().contains(&FAULT.to_string());
        }

        self.enable_motor();
    }

    // This enables the motor if it is currently not enabled
    pub fn enable_motor(&mut self) {
        if self.get_servo_status().contains(&MOTOR_ENABLED.to_string()) {
            return;
        }

        self.write_register(EXECUTE_COMMAND, 159);
        std::thread::sleep(time::Duration::from_millis(1000));
    }

    // This disables the motor if the motor is currently enabled
    pub fn disable_motor(&mut self) {
        if self.get_servo_status().contains(&MOTOR_ENABLED.to_string()) {
            self.write_register(EXECUTE_COMMAND, 158);
            std::thread::sleep(time::Duration::from_millis(1000));
        }
    }

    pub fn home_servo(&mut self) {
        self.reset_alarm_or_fault();

        // This will start the actual homing process
        println!("Starting to home servo: {}", self.servo_name);
        self.write_register(125, 1);
        std::thread::sleep(time::Duration::from_millis(1000));
        self.write_register(EXECUTE_COMMAND, 120);
        std::thread::sleep(time::Duration::from_millis(1000));

        // Now we wait until homing is complete or a timer expires and bail.
        let now = Instant::now();
        while self.get_servo_status().contains(&HOMING.to_string()) {
            println!("Servo status: {:?}", self.get_servo_status());
            if self.get_servo_status().contains(&ALARM.to_string()) {
                println!("Got alarm during homing.  Trying to reset.");
                self.reset_alarm_or_fault();
                println!("Restarting homing procedure.");
                self.write_register(125, 1);
                std::thread::sleep(time::Duration::from_millis(1000));
                self.write_register(EXECUTE_COMMAND, 120);
                std::thread::sleep(time::Duration::from_millis(1000));
            }
            // We will wait until max homing allowed time
            if now.elapsed().as_secs() > MAX_HOMING_TIME {
                println!("!!Unable to finish homing procedure!!");
                return;
            }
            std::thread::sleep(time::Duration::from_millis(300));
        }

        println!("Finished homing servo: {}", self.servo_name);
    }

    pub fn move_servo(&mut self, accel: u64, decel: u64, velocity: u64, encoder_position: u64) {
        if self.in_range(encoder_position) {
            return;
        }

        // Setup our two move portions
        let move1: u64 = encoder_position / MAX_32_BIT;
        let move2: u64 = encoder_position % MAX_32_BIT;

        println!(
            "Moving to position: {} (move 1: {}, move 2: {})",
            encoder_position, move1, move2
        );

        // Reset any possible faults, etc.
        self.reset_alarm_or_fault();

        // Setup the move parameter registers and let them settle
        self.write_register(ACCELERATION, accel);
        self.write_register(DECELERATION, decel);
        self.write_register(VELOCITY, velocity);
        self.write_register(DISTANCE_1, move1);
        self.write_register(DISTANCE_2, move2);
        std::thread::sleep(time::Duration::from_millis(25));

        println!(
            "D1: {}, D2: {}",
            self.get_register_value(DISTANCE_1),
            self.get_register_value(DISTANCE_2)
        );

        // This will start the actual move
        self.write_register(EXECUTE_COMMAND, 103);

        // We can wait until we are in position or freak out if we
        // have not made it in time.
        let now = Instant::now();
        while self.get_servo_status().contains(&MOVING.to_string()) {
            self.reset_alarm_or_fault();
            if self.get_servo_status().contains(&IN_POSITION.to_string()) {
                break;
            }
            if now.elapsed().as_secs() > MAX_MOVE_TIME {
                println!("!!Unable to finish requested move!!");
                break;
            }
            std::thread::sleep(time::Duration::from_millis(300));
            println!("Encoder count (MOVING): {}", self.get_encoder_count());
        }
        if !self.in_range(encoder_position) {
            println!(
                "Unable to reach requested encoder position of {} (actual: {})",
                encoder_position,
                self.get_encoder_count()
            );
        } else {
            println!("Encoder count (FINAL): {}", self.get_encoder_count(),);
        }
    }

    // Returns:
    //      TRUE if servo encoder position is with +/- range
    // based on value of ENCODER_POSITION_RANGE
    //      FALSE if it is not
    pub fn in_range(&mut self, requested_pos: u64) -> bool {
        let min_pos = requested_pos - ENCODER_POSITION_RANGE;
        let max_pos = requested_pos + ENCODER_POSITION_RANGE;
        let curr_pos: u64 = self.get_encoder_count();

        return curr_pos >= min_pos && curr_pos <= max_pos;
    }

    pub fn initialize(&mut self) {
        // TODO: Make this return bool true for success
        self.write_register(125, 1);
        std::thread::sleep(time::Duration::from_millis(1000));
        self.write_register(EXECUTE_COMMAND, 120);
        std::thread::sleep(time::Duration::from_millis(1000));
    }

    pub fn write_register(&mut self, register: u16, value: u64) {
        self.client
            .write_single_register(register, value as u16)
            .unwrap();
    }

    pub fn get_register_value(&mut self, register: u16) -> u64 {
        let ret = *self
            .client
            .read_holding_registers(register, 1)
            .expect("IO Error")
            .first()
            .unwrap_or(&0);

        ret as u64
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
    println!("Creating applied device: {}", servo_name);
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
        servo_status: Vec::new(),
        servo_alarm: Vec::new(),
    }
}
