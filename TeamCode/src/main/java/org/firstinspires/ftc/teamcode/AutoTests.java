package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Tests (Blocks to Java)", preselectTeleOp = "Robot2DriveV2")
public class AutoTests extends LinearOpMode {

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

  boolean showDriveTelemetry = false;

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
    leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        //main drive loop, methods here are called repeatedly while active
        driveStraight(1000);

        //Telemetry();
        autoStateCounter ++;
      }
    }
  }

  private void setMode () {
    rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  //difference between current and target
  private int posDiff(@NonNull DcMotor a) {
    return Math.abs(a.getCurrentPosition() - a.getTargetPosition());
  }

  private boolean posDiffTolerance(@NonNull DcMotor a, int tolerance) {
    return (Math.abs(a.getCurrentPosition() - a.getTargetPosition()) > tolerance);
  }

  private boolean anyMotorActive () {
    return (posDiffTolerance(leftFrontMotor, 5)  || posDiffTolerance(leftRearMotor, 5)
            || posDiffTolerance(rightFrontMotor, 5) || posDiffTolerance(rightRearMotor, 5));
  }

  private void powerRamp (DcMotor a, int distance) {
    if (posDiff(a) > (distance * (2/3))) {
      int progress = posDiff(a) - (distance * (2/3));
      a.setPower((1/progress) * masterSpeed);
    }
    else if (posDiff(a) < (distance * (1/3))) {
      int progress = (distance * (1/3) - posDiff(a));
      a.setPower((1/progress) * masterSpeed);
    }
    telemetry.addData("Target", a.getTargetPosition());
    telemetry.addData("Current", a.getCurrentPosition());
    telemetry.addData("distance", distance);
    telemetry.addData("posDiff", posDiff(a));
    telemetry.update();
  }
  private void reachTarget(boolean showTelemetry, int distance) {

    while (anyMotorActive()) {
      if (posDiff(leftFrontMotor) > 5) {
        powerRamp(leftFrontMotor, distance);
      }
      if (posDiff(leftRearMotor) > 5) {
        powerRamp(leftRearMotor, distance);
      }
      if (posDiff(rightFrontMotor) > 5) {
        powerRamp(rightFrontMotor, distance);
      }
      if (posDiff(rightRearMotor) > 5) {
        powerRamp(rightRearMotor, distance);
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


      telemetry.update();
    }
  }

  private void driveStraight(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() + distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() + distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + distance);

    setMode();

    reachTarget(showDriveTelemetry, distance);
  }

  private void driveBackwards(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() - distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() - distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - distance);

    setMode();

    reachTarget(showDriveTelemetry, distance);
  }

  private void turnRight(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() + distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() - distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - distance);

    setMode();

    reachTarget(showDriveTelemetry, distance);
  }

  private void turnLeft(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() - distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() + distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + distance);

    setMode();

    reachTarget(showDriveTelemetry, distance);
  }

  private void spinCarousel(int time) {
    rightCarousel.setPower(-0.3);
    leftCarousel.setPower(0.3);
    sleep(time);
    rightCarousel.setPower(0);
    leftCarousel.setPower(0);
  }

  private void strafeLeft(int time) {
    strafeMotor.setPower(1 * masterSpeed);
    sleep(time);
    strafeMotor.setPower(0);

  }

  private void strafeRight(int time) {
    strafeMotor.setPower(-1 * masterSpeed);
    sleep(time);
    strafeMotor.setPower(0);

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
