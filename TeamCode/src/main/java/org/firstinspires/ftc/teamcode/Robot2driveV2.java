package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot2driveV2 (Blocks to Java)")
public class Robot2driveV2 extends LinearOpMode {

  //initializing all motors and servos (making code recognize motors and assigning names)

  private ElapsedTime     runtime = new ElapsedTime();

  private DcMotor rightRearMotor;
  private DcMotor rightFrontMotor;
  private DcMotor leftRearMotor;
  private DcMotor leftFrontMotor;
  private DcMotor strafeMotor;
  private DcMotor chainMotor;

  private CRServo rightIntake;
  private CRServo leftIntake;
  private CRServo rightCarousel;
  private CRServo leftCarousel;

  //power to left/right side of drivetrain
  double leftMotorPower;
  double rightMotorPower;
  double strafePower;


  //drive constants, to control power
  double masterSpeed = 0.4;
  double turnSpeed = 0.3;
  double maxStrafePower = 0.5;

  String drivingMode = "a";

  double chainSpeed = 0.3;


  //values for strafe control


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
    chainMotor = hardwareMap.get(DcMotor.class, "chain_motor");

    //servo config
    rightIntake = hardwareMap.get(CRServo.class, "right_intake");
    leftIntake = hardwareMap.get(CRServo.class, "left_intake");
    rightCarousel = hardwareMap.get(CRServo.class, "right_carousel");
    leftCarousel = hardwareMap.get(CRServo.class, "left_carousel");

    //direction fixing (so all motors drive in the same direction)

    rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        moveChain();
        Telemetry();
      }
    }
  }

  // drive method, handles all driving-related behavior.
  private void driveMain() {
    speedBoost();
    drive(drivingMode);

    //assigning power values to motors
    rightRearMotor.setPower(rightMotorPower);
    rightFrontMotor.setPower(rightMotorPower);
    leftRearMotor.setPower(leftMotorPower);
    leftFrontMotor.setPower(leftMotorPower);
    strafeMotor.setPower(strafePower);
  }

  private void speedBoost() {
    //check if speed boost is enabled
    if (gamepad2.right_trigger > 0.5) {
      masterSpeed = 0.8;
    } else {
      masterSpeed = 0.4;
    }
  }

  private void drive(String mode) {
    if (mode.equals("dualJoystick")) {
      //left stick, used for driving straight
      if (-0.2 < gamepad2.right_stick_x && gamepad2.right_stick_x < 0.2) {
        leftMotorPower = masterSpeed * gamepad2.left_stick_y;
        rightMotorPower = masterSpeed * gamepad2.left_stick_y;
      }
      //right stick, used for turning
      if (-0.2 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.2) {
        leftMotorPower = turnSpeed * gamepad2.right_stick_x;
        rightMotorPower = turnSpeed * -gamepad2.right_stick_x;
      }
    }
    if (mode.equals("singleJoystick")) {
      //left stick used for driving straight and turning
      if (-0.2 < gamepad2.left_stick_x && gamepad2.left_stick_x < 0.2) {
        leftMotorPower = masterSpeed * gamepad2.left_stick_y;
        rightMotorPower = masterSpeed * gamepad2.left_stick_y;
      }

      if (-0.2 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.2) {
        leftMotorPower = turnSpeed * gamepad2.left_stick_x;
        rightMotorPower = turnSpeed * -gamepad2.left_stick_x;
      }
    }
    if (mode.equals("a")) {
      leftMotorPower = 0;
      rightMotorPower = 0;
      strafePower = 0;
      //left stick used for driving straight and strafing
      if (-0.2 > gamepad2.left_stick_y | gamepad2.left_stick_y > 0.2) {
        leftMotorPower = masterSpeed * gamepad2.left_stick_y;
        rightMotorPower = masterSpeed * gamepad2.left_stick_y;
      }
      //right stick used for turning
      if (-0.2 > gamepad2.right_stick_x | gamepad2.right_stick_x > 0.2) {
        leftMotorPower = turnSpeed * gamepad2.right_stick_x;
        rightMotorPower = turnSpeed * -gamepad2.right_stick_x;
      }
      //strafing
      if (-0.2 > gamepad2.left_stick_x | gamepad2.left_stick_x > 0.2) {
        strafePower = maxStrafePower * -gamepad2.left_stick_x;
      }
    }
  }

  //strafe code, used in drive method (currently not used due to new drive style)
  private void Strafe() {
    if (gamepad2.left_bumper) {
      strafePower = maxStrafePower * 1;
    } else if (gamepad2.right_bumper) {
      strafePower = maxStrafePower * -1;
    }
    else {
      strafePower = 0;
    }
  }

  //intake code, to take in cargo.
  private void Intake() {
    if (gamepad1.right_trigger > 0.1) {
      intakePower(gamepad1.right_trigger);
    } else if(gamepad1.left_trigger > 0.1) {
      intakePower(gamepad1.left_trigger * -1);
    } else {
      intakePower(0);
    }
  }

  //set intake power
  private void intakePower(float d) {
    rightIntake.setPower(d);
    leftIntake.setPower(d);
  }

  private void Carousel() {
    if (gamepad1.a) {
      leftCarousel.setPower(0.7);
      rightCarousel.setPower(-0.7);
    } else {
      leftCarousel.setPower(0);
      rightCarousel.setPower(0);
    }
  }

  private void moveChain() {
    if (gamepad2.right_bumper) {
      chainMotor.setPower(1*chainSpeed);
    } else if(gamepad2.left_bumper) {
      chainMotor.setPower(-1*chainSpeed);
    } else {
      chainMotor.setPower(0);
    }
  }

  //telemetry updates, to see info live, while robot is active
  private void Telemetry() {
    telemetry.addLine("Bartholomew V2");
    telemetry.addData("Chain pow", chainMotor.getPower());
    telemetry.update();
  }
}