package org.firstinspires.ftc.teamcode;

//simple automode, only using strafe, blue side

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "StrafeAutoBlue (Blocks to Java)", preselectTeleOp = "Robot2DriveV2")
public class StrafeAutoBlue extends LinearOpMode {

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

  String autoState = "idle";
  int autoStateCounter = 0;
  double masterSpeed = 0.5;

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
        parked();
        Telemetry();
        autoStateCounter ++;
      }
    }
  }

  private void carouselApproach() {
    if (autoStateCounter == 0) {
      autoState = "carouselApproach";
      strafeRight(5000);
    }
  }


  private void carouselScore() {
    if (autoStateCounter == 1) {
      autoState = "carouselScore";
      spinCarousel();
    }
  }

  private void warehouseTransit() {
    if (autoStateCounter == 2) {
      autoState = "warehouseTransit";
      strafeLeft(1500);
    }
  }

  private void warehouseParking() {
    if (autoStateCounter == 3) {
      autoState = "warehouseParking";
      strafeLeft(7000);
    }
  }

  private void parked() {
    if (autoStateCounter == 4) {
      autoState = "parked!";
      driveStraight(5000);
    }
  }

  private void setPower(double pow) {
    rightRearMotor.setPower(pow * masterSpeed);
    rightFrontMotor.setPower(pow * masterSpeed);
    leftRearMotor.setPower(pow * masterSpeed);
    leftFrontMotor.setPower(pow * masterSpeed);
  }

  private void setPowerLeft(double pow) {
    leftRearMotor.setPower(pow * masterSpeed);
    leftFrontMotor.setPower(pow * masterSpeed);
  }

  private void setPowerRight(double pow) {
    rightRearMotor.setPower(pow * masterSpeed);
    rightFrontMotor.setPower(pow * masterSpeed);
  }

  private void driveStraight(int time) {
    setPower(1);
    sleep(time);
    setPower(0);
    sleep(250);
  }

  private void driveBackwards(int time) {
    setPower(-1);
    sleep(time);
    setPower(0);
    sleep(250);
  }

  private void turnLeft(int time) {
    setPowerLeft(1);
    setPowerRight(-1);
    sleep(time);
    setPower(0);
    sleep(250);
  }

  private void turnRight(int time) {
    setPowerLeft(-1);
    setPowerRight(1);
    sleep(time);
    setPower(0);
    sleep(250);
  }

  private void spinCarousel() {
    rightCarousel.setPower(-0.5);
    leftCarousel.setPower(0.5);
    sleep(3500);
    rightCarousel.setPower(0);
    leftCarousel.setPower(0);
  }

  private void strafeLeft(int time) {
    strafeMotor.setPower(0.5);
    sleep(time);
    strafeMotor.setPower(0);
  }

  private void strafeRight(int time) {
    strafeMotor.setPower(-0.5);
    sleep(time);
    strafeMotor.setPower(0);
  }

  private void Telemetry() {

    telemetry.addData("State", autoState);
    telemetry.addData("StateCt", autoStateCounter);
    telemetry.addData("Left", leftMotorPower);
    telemetry.addData("Right", rightMotorPower);
    telemetry.addData("Masterspeed", masterSpeed);
    telemetry.update();
  }
}

