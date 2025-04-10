# What needs to be done for the Turret: #

## Adding the devices
    1. Add in the hardware from the turret CAD into the RobotManager and HardwareDevices classes
        There should be:
            - A turret master flywheel motor (DcMotorImplEx)
            - A turret slave flywheel motor (DcMotorImplEx)
            - A turret rotation motor (DcMotorImplEx)
            - An intake motor (DcMotorImplEx)
            - A turret intake servo (CRServoImplEx)
            - A lateral conveyer servo (CRServoImplEx)
            - A longitudinal conveyer servo (CRServoImplEx)
            - A turret arc servo (ServoImplEx)
            - A REV Absolute Encoder (idk what type)

## Create a turret class
    1. Create a turret class that extends hardware devices so that you can use the hardware devices from the RobotManager class

    2. Write a constrtuctor inside that class that takes in the RobotManager class so that you can use other functions from the codebase

    3. Add the class to the bottom of the RobotManager class in the same format of the other subsystems that were added,
    this will create the class as an object so that you will be able to refrence it in other classes through robotManager.turret

## Create an intake class
    1. Follow the same instructions as above, this will contain the intake motor and conveyer servos

## Creating subclasses
    1. Create a subclass in the Turret subsystem class labelled TurretFlywheel
        - This is because there are two motors that are going to be controlled ALWAYS at the same time and at the same speed,
        so it would be redundant to have to set the power twice. When doing anything with these motors, you should tell the master motor
        to do something, and then tell the slave motor to do what the master motor is doing. This prevents them from ever doing something different
        and breaking a belt or the other motor, and is a good concept to learn for the future.

    2. Create a subclass in the Intake subsystem class labelled ConveyerBelts

## Implementing functions
    1. Create functions inside of the subsystems and subclasses, and remember that functions are not required to be inside a subclass

    2. Function suggestions:
        Turret:
            - rotateTo(degrees) // Clockwise should be positive, use the absolute encoder
            - intake(power)
            - setArcAngle(degrees)

        TurretFlywheel:
            - setPower(power)

        Intake:
            - intake(power) DONE

        ConveyerBelts:
            - setLateralPower(power)
            - setLongitudinalPower(power)

## Final touches
    1. When making these, you should need to add variables. If there are constant values that are being used,
    put those into the constants file and refrence them from there. Other variables can be placed right below where you define the class.

    2. You may also find something where you can update the status of, such as intakeIsIntaking (don't do that one). If so,
    put that status into the status file, and update it as it changes in your code.

    3. If you need to reverse motors or reset their encoders, do it in the RobotManager

    4. If you want to use the indicator light then you can use that and have it be a color when something happens

    5. The final step is limelight, we can talk about that later<3