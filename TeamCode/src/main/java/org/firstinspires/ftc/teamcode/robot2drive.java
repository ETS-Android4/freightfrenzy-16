package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "robot2drive2 (Blocks to Java)")
public class robot2drive2 extends LinearOpMode {

  private DcMotor right_rear_motor;
  private DcMotor right_front_motor;
  private DcMotor left_rear_motor;
  private DcMotor left_front_motor;
  private DcMotor strafe_motor;
  private CRServo right_intake;
  private CRServo left_intake;


  double L;
  double R;
  double masterSpeed = 0.4;
  double turnSpeed = 0.3;
  double strafe_increment = 0.001;
  double strafe_pow = 0;
  double max_strafe_pow = 0.4;

  @Override
  public void runOpMode() {
    right_rear_motor = hardwareMap.get(DcMotor.class, "right_rear_motor");
    right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
    left_rear_motor = hardwareMap.get(DcMotor.class, "left_rear_motor");
    left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
    strafe_motor = hardwareMap.get(DcMotor.class, "strafe_motor");

    right_intake = hardwareMap.get(CRServo.class, "right_intake");
    left_intake = hardwareMap.get(CRServo.class, "left_intake");

    right_rear_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    right_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    left_rear_motor.setDirection(DcMotorSimple.Direction.REVERSE);

    right_intake.setDirection(DcMotorSimple.Direction.REVERSE);

    right_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        Drive();
        Intake();
        Telemetry();
      }
    }
  }

// drive method, handles all driving-related behavior.
  private void Drive() {
    if (gamepad2.right_trigger > 0.5) {
        masterSpeed = 0.8;
    } else {
        masterSpeed = 0.4;
    }

    if (-0.2 < gamepad2.right_stick_x && gamepad2.right_stick_x < 0.2) {
      L = masterSpeed * gamepad2.left_stick_y;
      R = masterSpeed * gamepad2.left_stick_y;
    }
    if (-0.2 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.2) {
      L = turnSpeed * -gamepad2.right_stick_x;
      R = turnSpeed * gamepad2.right_stick_x;
    }
    if (gamepad2.left_bumper) {
      Strafe(1);
    } else if (gamepad2.right_bumper) {
      Strafe(-1);
    } else {
      strafe_pow = 0;
    }

    right_rear_motor.setPower(R);
    right_front_motor.setPower(R);
    left_rear_motor.setPower(L);
    left_front_motor.setPower(L);
    strafe_motor.setPower(strafe_pow);
  }

//strafe code, used in drive method.
  private void Strafe(int goal) {
    if (goal == 1 && strafe_pow < max_strafe_pow) {
        strafe_pow += (strafe_increment * goal);
    } 
    if (goal == -1 && strafe_pow > (max_strafe_pow * -1)) {
        strafe_pow += (strafe_increment * goal);
    }
  }

//intake code, to take in cargo.
  private void Intake() {
    if (gamepad2.x) {
      Intake_power(1);
    } else if(gamepad2.y) {
      Intake_power(-1);
    } else {
      Intake_power(0);
    }
  }

  private void Intake_power(int d) {
    right_intake.setPower(d);
    left_intake.setPower(d);
  }


  private void Telemetry() {
    telemetry.addData("Strafe", strafe_pow);
    telemetry.addData("intakeleft", left_intake);
    telemetry.addData("intakeright", right_intake);
    telemetry.addData("Left", L);
    telemetry.addData("Right", R);
    telemetry.update();
  }
}
