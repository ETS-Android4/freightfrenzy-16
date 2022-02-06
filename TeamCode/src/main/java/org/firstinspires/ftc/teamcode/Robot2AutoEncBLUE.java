package org.firstinspires.ftc.teamcode;

//automode with encoders, currently nonfunctional due to issues w leftrearmotor

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Robot2AutoEncBLUE (Blocks to Java)", preselectTeleOp = "Robot2DriveV2")
public class Robot2AutoEncBLUE extends LinearOpMode {

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

  String autoState = "idle";
  int autoStateCounter = 0;
  double masterSpeed = 0.5;

  boolean showDriveTelemetry = true;

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

    rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //opMode loop, this is what actually runs when you press start.
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        driveStraight(1000);
        //main drive loop, methods here are called repeatedly while active


        //Telemetry();
        autoStateCounter ++;
      }
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

  private void setMode () {
    rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    strafeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

  }

  private void reachTarget(boolean showTelemetry) {

    setMode();

    while (posDiff(leftFrontMotor) || posDiff(leftRearMotor) || posDiff(rightFrontMotor) || posDiff(rightRearMotor)) {
      if (posDiff(leftFrontMotor)) {
        leftFrontMotor.setPower(masterSpeed);
      }
      if (posDiff(leftRearMotor)) {
        leftRearMotor.setPower(masterSpeed);
      }
      if (posDiff(rightFrontMotor)) {
        rightFrontMotor.setPower(masterSpeed);
      }
      if (posDiff(rightRearMotor)) {
        rightRearMotor.setPower(masterSpeed);
      }
      if (posDiff(strafeMotor)) {
        strafeMotor.setPower(masterSpeed);
      }
    }
    if (showTelemetry) {
      telemetry.addData("LRTarget", leftRearMotor.getTargetPosition());
      telemetry.addData("LRCurrent", leftRearMotor.getCurrentPosition());
      telemetry.addData("LRpower", leftRearMotor.getPower());

      telemetry.addData("LFTarget", leftFrontMotor.getTargetPosition());
      telemetry.addData("LFCurrent", leftFrontMotor.getCurrentPosition());
      telemetry.addData("LFpower", leftFrontMotor.getPower());

      telemetry.addData("RFTarget", rightFrontMotor.getTargetPosition());
      telemetry.addData("RFCurrent", rightFrontMotor.getCurrentPosition());
      telemetry.addData("RFpower", rightFrontMotor.getPower());

      telemetry.addData("RRTarget", rightRearMotor.getTargetPosition());
      telemetry.addData("RRCurrent", rightRearMotor.getCurrentPosition());
      telemetry.addData("RRpower", rightRearMotor.getPower());

      telemetry.addData("STRAFETarget", strafeMotor.getTargetPosition());
      telemetry.addData("STRAFECurrent", strafeMotor.getCurrentPosition());
      telemetry.addData("STRAFEpower", strafeMotor.getPower());

      telemetry.update();
    }
  }

  private boolean posDiff(DcMotor a) {
    return Math.abs(a.getCurrentPosition() - a.getTargetPosition()) > 10;

  }

  private void driveStraight(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() + distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() + distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + distance);

    reachTarget(showDriveTelemetry);
  }

  private void driveBackwards(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() - distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() - distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - distance);

    reachTarget(showDriveTelemetry);
  }

  private void turnLeft(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() + distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() - distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - distance);

    reachTarget(showDriveTelemetry);
  }

  private void turnRight(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() - distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() + distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + distance);

    reachTarget(showDriveTelemetry);
  }

  private void spinCarousel(int time) {
    rightCarousel.setPower(-0.5);
    leftCarousel.setPower(0.5);
    sleep(time);
    rightCarousel.setPower(0);
    leftCarousel.setPower(0);
  }

  private void strafeLeft(int distance) {
    strafeMotor.setTargetPosition(strafeMotor.getCurrentPosition() + distance);

    reachTarget(showDriveTelemetry);

  }

  private void strafeRight(int distance) {
    strafeMotor.setTargetPosition(strafeMotor.getCurrentPosition() - distance);

    reachTarget(showDriveTelemetry);

  }

  private void Telemetry() {

    telemetry.addData("State", autoState);
    telemetry.addData("StateCt", autoStateCounter);
    telemetry.addData("Target", leftRearMotor.getTargetPosition());
    telemetry.addData("Current", leftRearMotor.getCurrentPosition());
    telemetry.addData("Masterspeed", masterSpeed);
    telemetry.update();
  }
}
