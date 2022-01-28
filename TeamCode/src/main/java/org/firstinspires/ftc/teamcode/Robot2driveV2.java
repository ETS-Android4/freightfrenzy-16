package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot2driveV2 (Blocks to Java)")
public class Robot2driveV2 extends LinearOpMode {

  private ElapsedTime     runtime = new ElapsedTime();

  private DcMotor rightRearMotor;
  private DcMotor rightFrontMotor;
  private DcMotor leftRearMotor;
  private DcMotor leftFrontMotor;
  private DcMotor strafeMotor;

  private CRServo rightIntake;
  private CRServo leftIntake;
  private CRServo rightCarousel;
  private CRServo leftCarousel;

  //power to left/right side of drivetrain
  double leftMotorPower;
  double rightMotorPower;

  //drive constants, to control power
  double masterSpeed = 0.4;
  double turnSpeed = 0.3;

  String drivingMode = "dualJoystick";
  double driveModeToggleWait = 0;

  //values for strafe control
  double strafeIncrement = 0.001;
  double currentStrafePower = 0;
  double maxStrafePower = 0.4;

  double strafeIncrementWait = 0;

  @Override
  public void runOpMode() {
    //hardware configuration, naming all motors/servos and configuring direction/behaviour

    //motor config
    rightRearMotor = hardwareMap.get(DcMotor.class, "right_rear_motor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
    leftRearMotor = hardwareMap.get(DcMotor.class, "left_rear_motor");
    leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
    strafeMotor = hardwareMap.get(DcMotor.class, "strafe_motor");

    //servo config
    rightIntake = hardwareMap.get(CRServo.class, "right_intake");
    leftIntake = hardwareMap.get(CRServo.class, "left_intake");
    rightCarousel = hardwareMap.get(CRServo.class, "right_carousel");
    leftCarousel = hardwareMap.get(CRServo.class, "left_carousel");

    //direction fixing (so all motors drive in the same direction)
    leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    //prevent robot from rolling when power is cut
    rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //opMode loop, this is what actually runs when you press start.
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        //main drive loop, methods here are called repeatedly while active
        driveMain();
        Intake();
        Carousel();
        Telemetry();
      }
    }
  }

  // drive method, handles all driving-related behavior.
  private void driveMain() {
    speedBoost();
    toggleDriveMode();
    driveCheck(drivingMode);

    //strafing
    checkStrafePowerChange();
    Strafe();

    //assigning power values to motors
    rightRearMotor.setPower(rightMotorPower);
    rightFrontMotor.setPower(rightMotorPower);
    leftRearMotor.setPower(leftMotorPower);
    leftFrontMotor.setPower(leftMotorPower);
    strafeMotor.setPower(currentStrafePower);
  }

  private void speedBoost() {
    //check if speed boost is enabled
    if (gamepad2.right_trigger > 0.5) {
      masterSpeed = 0.8;
    } else {
      masterSpeed = 0.4;
    }
  }

  private void toggleDriveMode() {
    //changes between single joystick drive and dual joystick drive
    if (gamepad2.left_trigger > 0.5 && runtime.seconds() > driveModeToggleWait) {
      if (drivingMode.equals("dualJoystick")) {
        drivingMode = "singleJoystick";
      }
      else {
        drivingMode = "dualJoystick";
      }
      driveModeToggleWait = runtime.seconds() + 0.5;
    }
  }

  private void driveCheck(String mode) {
    if (mode.equals("dualJoystick")) {
      //right stick, used for driving straight
      if (-0.2 < gamepad2.right_stick_x && gamepad2.right_stick_x < 0.2) {
        leftMotorPower = masterSpeed * gamepad2.left_stick_y;
        rightMotorPower = masterSpeed * gamepad2.left_stick_y;
      }
      //left stick, used for turning
      if (-0.2 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.2) {
        leftMotorPower = turnSpeed * -gamepad2.right_stick_x;
        rightMotorPower = turnSpeed * gamepad2.right_stick_x;
      }
    }
    if (mode.equals("singleJoystick")) {
      //right stick used for driving straight and turning
      if (-0.2 < gamepad2.left_stick_x && gamepad2.left_stick_x < 0.2) {
        leftMotorPower = masterSpeed * gamepad2.left_stick_y;
        rightMotorPower = masterSpeed * gamepad2.left_stick_y;
      }

      if (-0.2 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.2) {
        leftMotorPower = turnSpeed * -gamepad2.left_stick_x;
        rightMotorPower = turnSpeed * gamepad2.left_stick_x;
      }
    }
  }

  //strafe code, used in drive method.
  private void Strafe() {
    if (gamepad2.right_bumper) {
      if (currentStrafePower < maxStrafePower) {
        //if strafe power isn't at max, increment
        currentStrafePower += strafeIncrement;
      }
    } else if (gamepad2.left_bumper) {
      if (currentStrafePower > (maxStrafePower * -1)) {
        //same for other direction
        currentStrafePower += strafeIncrement * -1;
      }
    } else {
      currentStrafePower = 0;
    }
  }

  private void checkStrafePowerChange(){
    if (gamepad2.a) {
      strafePowerChange("Up");
    }
    if (gamepad2.b) {
      strafePowerChange("Down");
    }
  }

  private void strafePowerChange(String change) {
    if (runtime.seconds() > strafeIncrementWait) {
      if (change.equals("Up") && maxStrafePower < 1) {
        maxStrafePower += 0.1;
        strafeIncrementWait = runtime.seconds() + 0.5;
      }
      if (change.equals("Down") && maxStrafePower > 0.2) {
        maxStrafePower -= 0.1;
        strafeIncrementWait = runtime.seconds() + 0.5;
      }
    }
  }

  //intake code, to take in cargo.
  private void Intake() {
    if (gamepad2.x) {
      intakePower(1);
    } else if(gamepad2.y) {
      intakePower(-1);
    } else {
      intakePower(0);
    }
  }

  //set intake power
  private void intakePower(int d) {
    rightIntake.setPower(d);
    leftIntake.setPower(d);
  }

  private void Carousel() {
    if (gamepad2.dpad_left) {
      leftCarousel.setPower(1);
    } else if (gamepad2.dpad_right) {
      rightCarousel.setPower(-1);
    } else {
      leftCarousel.setPower(0);
      rightCarousel.setPower(0);
    }
  }

  //telemetry updates, to see info live, while robot is active
  private void Telemetry() {
    telemetry.addData("Strafe", currentStrafePower);
    telemetry.addData("Strafe Max", maxStrafePower);
    telemetry.addData("Left", leftMotorPower);
    telemetry.addData("Right", rightMotorPower);
    telemetry.addData("Drive Mode", drivingMode);
    telemetry.update();
  }
}
