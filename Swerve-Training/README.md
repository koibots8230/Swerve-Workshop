# Swerve Module Pose Estimation

## Goal
Instead of updating the robot position based on joystick inputs, we want to estimate our position using information provided by the swerve modules.

## Recommended Readings
- [WPILib State Space Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [SwerveDrivePoseEstimator API Documentation](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator.html)

---

## Step 1: Create the SwerveDrivePoseEstimator

In the `Swerve` class of `Swerve.java`, create a `SwerveDrivePoseEstimator` instance.

### Constructor Requirements
The `SwerveDrivePoseEstimator` constructor takes 4 arguments:
1. **Kinematics** - Your swerve drive kinematics
2. **Gyro** - Your gyroscope reading
3. **Module Positions** - The position of each module
4. **Initial Pose** - Your starting pose

We have everything we need except a method to get the module positions.

---

## Step 2: Create Module Position Method

In `SwerveModule.java`, we'll create a method that returns the position of individual modules.

### Create New Variables
Create two new variables:
- **position** - Where each module is located
- **angle** - The direction the module is facing

### Update setState Method
In your `setState` method:
1. Create a **driveSetpointVelocity** using information from the methods parameter
2. Create a **turnSetpointAngle** using information from the methods parameter
3. Use both setpoints to update your position and angle variables
   - **position** should track where each module is
   - **angle** should track the direction it's facing

### Create getPosition Method
Create a new method that returns a `SwerveModulePosition`:
```java
public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(position, angle);
}
```

---

## Step 3: Get All Module Positions

In `Swerve.java`, create a method that stores an array of all 4 swerve module positions. It should return an array of SwerveModulePosition[]

Use this method in your `SwerveDrivePoseEstimator` constructor to provide the module positions.

🎉 **It's constructed! Pat yourself on the back.**

---

## Step 4: Update Field-Relative Drive

In your field-relative drive method:
1. Use your `SwerveDrivePoseEstimator` to get the estimated position
2. At the end of the method, update the estimator with:
   - Your gyro
   - The new module positions method you just created

This ensures the pose estimator continuously updates with fresh data from your modules.