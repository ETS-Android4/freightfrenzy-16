package org.firstinspires.ftc.teamcode;

//automode red side

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Robot2AutonomousV1 (Blocks to Java)")
public class Robot2AutonomousV1 extends LinearOpMode {

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
        //main drive loop, methods here are called repeatedly while active
        carouselApproach();
        carouselScore();
        warehouseTransit();
        warehouseParking();
        parked();

        Telemetry();
        autoStateCounter ++;
      }
    }
  }

  private void carouselApproach() {
    if (autoStateCounter == 0) {
      autoState = "carouselApproach";
      driveStraight(100);
      turnLeft(400);
      driveStraight(1000);
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
      driveBackwards(2000);
      //align

    }
  }

  private void warehouseParking() {
    if (autoStateCounter == 3) {
      autoState = "warehouseParking";
    }
  }

  private void parked() {
    if (autoStateCounter == 4) {
      autoState = "parked!";
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
  }

  private void driveBackwards(int time) {
    setPower(-1);
    sleep(time);
    setPower(0);
  }

  private void turnLeft(int time) {
    setPowerLeft(1);
    setPowerRight(-1);
    sleep(time);
    setPower(0);
  }

  private void turnRight(int time) {
    setPowerLeft(-1);
    setPowerRight(1);
    sleep(time);
    setPower(0);
  }

  private void spinCarousel() {
    //this
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
