# Swerve Drive Implementation Guide

## 🎯 Goal
We're building a working swerve drive! Here's our roadmap:
- Update constants to match AnglerFish configuration
- Integrate real gyro hardware
- Separate real robot code from simulation
- Configure motors and encoders
- Implement setpoint tracking and controllers
- Add trapezoidal motion profiling
- Apply miscellaneous optimizations

---

## 📚 Recommended Reading
Before diving in, familiarize yourself with these resources:
- [TrapezoidProfile JavaDoc](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrapezoidProfile.html)
- [Trapezoidal Profiles Guide](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html)
- [Turret Example (KoiBots)](https://github.com/koibots8230/Programming-Conventions/blob/main/ExampleProject/src/main/java/frc/robot/subsystems/Turret.java)
- [Gyro Software Guide](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html)
- [Programming Swerve Drive](https://dominik.win/blog/programming-swerve-drive/)

---

## Updating Constants
- Use values from the **AnglerFish** robot to update your code (obviously)

---

## 🧭 Gyro Integration
**Add the Pigeon 2 Gyro to `Swerve.java`:**
- Device ID: `10`
- Replace `estimatedPose` calculations with actual gyro readings

---

## Separate Real from Simulation

### In `Swerve.java`:
- Add `periodic()` method for real robot execution
- Add `simulationPeriodic()` method for simulation
- Call corresponding `SwerveModule` methods from each

### In `SwerveModule.java`:
- Add `periodic()` method
- Add `simulationPeriodic()` method
- Add the required information to each method (obviously)

**Pattern:** In `Swerve.periodic()` → call `SwerveModule.periodic()`  
Same pattern for simulation methods

---

## Motor and Encoder Configuration

### Create Hardware Objects:
- Drive motor
- Turn motor
- Related encoders
- Motor configurations
- Motor controllers

### Configuration Steps:
1. **Construct** motors, encoders, controllers, and turn config
2. **Set motor idle mode** to `brake`
3. **Set smart current limits:**
   - Drive: `30A`
   - Turn: `60A`
4. **Turn motor inversion:** `false`
5. **Set position and velocity conversion factors** (obviously)
6. ** CRITICAL: SET TURN ABSOLUTE ENCODER TO INVERTED!**
7. **Configure turn motor:**
```java
   turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

### Logging:
- Create all necessary logged values (obviously)

---

## Setting and Updating Values

### Logged Values:
- Remember those logged values? Construct them at the bottom of the constructor
- Copy and paste the same code into `periodic()` to update them

### Offset Handling:

#### What is an offset?
When we zero the modules, they're not all aligned in the same orientation. That's why we need offsets.

**Implementation:**
- Subtract an offset from `turnMotorPosition`
- Look in AnglerFish code to find these offsets
- Derive a way to set the correct offset in `SwerveModule.java` based on motor ID

---

## setState Method

### Optimization:
1. **Optimize your state** (comment this line out though—not sure why it breaks the code. If you figure it out, you'd surpass me in this)
2. **Apply cosine scaling**
3. **Update setpoints:**
   - `driveSetpoint` using `state`
   - `turnSetpoint` using `state`
4. **Control output:**
   - `driveController.setReference()` goes here

---

## PID Configuration

### Turn PID:
- Configure closed-loop turn PID with a P value
- Add I and D if desired don't know why you would tho
- Add feedback sensor
- **Enable position wrapping**
- **Set input range:** `-π` to `π`

### Drive PID:
- Configure closed-loop drive PIDF
- Set P and kV values
- Add I and D if desired ig

---

## Capstone: Trapezoidal Profile Subsystem

**This is your challenge!**
- Implement the trapezoidal motion profile system yourself
- Use the documentation and example code provided above
- This is a good test of your ability to figure things out independently
- Also not my job to teach you, you should already know this
- **Remember to have fun btw!**
- Anyone who complains loses a point.
- im serious

---

## Optimizations

### Input Scaling:
- It's possible to scale inputs for better deadband handling
- I don't do this—check Sailfish code if interested

### Deadband Application:
- Add `MathUtil.applyDeadband()` to your `x`, `y`, and `z` inputs in the drive command
---