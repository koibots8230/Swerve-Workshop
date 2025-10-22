# Sim Gyro

## Goals
1. Update heading based on alliance color  
2. Add a method to zero the simulated gyro  

---

### Suggested Readings
1. [Chassis Speeds and Odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html)

---

### Sim Gyro

The purpose of the simulated gyro is to **track heading** and **adjust orientation** depending on the alliance color. This ensures that the robot’s simulated pose correctly reflects its direction on the field.

---

#### Step 1: Create a Simulated Heading
1. Inside your `Swerve` class, create a new `Rotation2d` variable — name it something like `simHeading`.  
2. Initialize it in the constructor. The initial value doesn’t matter, so you can leave it blank or set it to `new Rotation2d()`.

---

#### Step 2: Adjust Heading by Alliance
In the method where you pass the alliance color from `RobotContainer` into `Swerve`:
- If the alliance is **Red**, set `simHeading` to **π** (180°).  
- If the alliance is **Blue**, set `simHeading` to **0**.  

This keeps your simulated robot facing the correct direction relative to the field.

---

#### Step 3: Integrate Heading into Pose
To use your simulated heading:
1. Replace your use of `omega` with `simHeading`.  
2. Set simHeading equal to inself and add the change in roation every clock cycle. 
3. Remeber that FRC types don't allow you to use opperators (+,-,*,/) instead you need to use the inbuilt methods. 

This ensures that the pose rotates correctly as the robot “turns” in simulation.

---

### Zeroing the Gyro

#### Step 1: Create a Reset Method
Inside `Swerve.java`, create a new method (e.g. `zeroSimGyro()`) that sets the rotation to **0 radians**.

#### Step 2: Usage Note
> **Important:**  
> This method should **never** be used during a match or mapped to a driver button.  
> It exists **only** for autonomous setup or simulation testing.

---

If you’ve done everything correctly, your robot’s simulated gyro will **track orientation accurately** and **reset cleanly when needed!**
