package org.firstinspires.ftc.teamcode;

//automode with encoders, blue side
// to be renamed

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
        carouselScore();
        warehouseTransit();

        //main drive loop, methods here are called repeatedly while active


        //Telemetry();
        autoStateCounter ++;
      }
    }
  }

  //autonomous stages, goes through one by one and completes tasks
  //simply to divide code well, not super important
  private void carouselScore() {
    if (autoStateCounter == 0) {
      autoState = "carouselApproach";
      strafeRight(300);
      driveStraight(950);
      spinCarousel(3000);


    }
  }


  private void warehouseTransit() {
    if (autoStateCounter == 1) {
      autoState = "warehouseTransit";
      driveBackwards(400);
      turnRight(900);
      driveStraight(2700);
      turnRight(70);
      strafeRight(800);


    }
  }

  private void warehouseParking() {
    if (autoStateCounter == 2) {
      autoState = "warehouseParking";

    }
  }

  private void parked() {
    if (autoStateCounter == 3) {
      autoState = "parked!";
    }
  }

  private void setMode () {
    rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  //difference between current and target
  private boolean posDiff(DcMotor a) {
    return Math.abs(a.getCurrentPosition() - a.getTargetPosition()) > 30;
  }

  private void reachTarget(boolean showTelemetry) {

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

    reachTarget(showDriveTelemetry);
  }

  private void driveBackwards(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() - distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() - distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - distance);

    setMode();

    reachTarget(showDriveTelemetry);
  }

  private void turnRight(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() + distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() - distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() - distance);

    setMode();

    reachTarget(showDriveTelemetry);
  }

  private void turnLeft(int distance) {
    rightRearMotor.setTargetPosition(rightRearMotor.getCurrentPosition() - distance);
    rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - distance);
    leftRearMotor.setTargetPosition(leftRearMotor.getCurrentPosition() + distance);
    leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + distance);

    setMode();

    reachTarget(showDriveTelemetry);
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
