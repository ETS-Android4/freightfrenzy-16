package org.firstinspires.ftc.teamcode;

//template for autonomous when needed

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot2AutonomousV1 (Blocks to Java)")
public class Robot2AutonomousV1 extends LinearOpMode {

  private ElapsedTime     runtime = new ElapsedTime();

  private DcMotor rightRearMotor;
  private DcMotor rightFrontMotor;
  private DcMotor leftRearMotor;
  private DcMotor leftFrontMotor;
  private DcMotor strafeMotor;

  private CRServo rightIntake;
  private CRServo leftIntake;

  //power to left/right side of drivetrain
  double leftMotorPower;
  double rightMotorPower;

  String autoState = "idle";
  int autoStateCounter = 0;

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
        one();
        two();
        three();
        four();
        five();

        autoStateCounter ++;
      }
    }
  }

  private void one() {
    if (autoStateCounter == 0) {

    }
  }

  private void two() {
    if (autoStateCounter == 1) {

    }
  }

  private void three() {
    if (autoStateCounter == 2) {

    }
  }

  private void four() {
    if (autoStateCounter == 3) {

    }
  }

  private void five() {
    if (autoStateCounter == 4) {

    }
  }

  private void setPower(double pow) {
    rightRearMotor.setPower(pow);
    rightFrontMotor.setPower(pow);
    leftRearMotor.setPower(pow);
    leftFrontMotor.setPower(pow);
  }

  private void driveStraight(int time) {
    setPower(0.5);
    sleep(time);
    setPower(0);
  }

  private void turnLeft(int time) {
    setPower(0.5);
    sleep(time);
    setPower(0);
  }

  private void turnRight(int time) {
    setPower(0.5);
    sleep(time);
    setPower(0);
  }

  private void driveBackwards(int time) {
    setPower(0.5);
    sleep(time);
    setPower(0);
  }

  private void Telemetry() {

    telemetry.addData("State", autoState);
    telemetry.addData("StateCt", autoStateCounter);
    telemetry.addData("Left", leftMotorPower);
    telemetry.addData("Right", rightMotorPower);
    telemetry.update();
  }


}
