package frc.robot.pathfinding;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Titan;

public class Mimic {
	public static final String mimicFile = "/media/sda1/%s.mimic";
	public static final String formatString = "%.2f,%.2f,%.2f,%.4f,%.4f,%d,%.2f,%.2f,%.2f\n"; //LEFT ENCODER, RIGHT ENCODER, GYRO ANGLE, LEFT POWER, RIGHT POWER, HOME, ELEVATOR_HEIGHT, INTAKE_TILT, INTAKE_SPEED
	
	public static class Stepper {
		public double leftDistance, rightDistance, angle, leftPower, rightPower, elevatorHeight, intakeTilt, intakeSpeed;
		public boolean isHome;
		
		public Stepper(final double lD, final double rD, final double a, final double lP, final double rP, final boolean h, final double eH, final double iT, final double iS) {
			leftDistance = lD;
			rightDistance = rD;
			angle = a;
			leftPower = lP;
			rightPower = rP;
			isHome = h;
			elevatorHeight = eH;
			intakeTilt = iT;
			intakeSpeed = iS;
		}
		
		public Stepper(final String toParse) {
			try {
				final String parts[] = toParse.split(",");
				leftDistance = getDouble(parts[0]);
				rightDistance = getDouble(parts[1]);
				angle = getDouble(parts[2]);
				leftPower = getDouble(parts[3]);
				rightPower = getDouble(parts[4]);
				isHome = getBoolean(parts[5]); 
				elevatorHeight = getDouble(parts[6]);
				intakeTilt = getDouble(parts[7]);
				intakeSpeed = getDouble(parts[8]);
			} catch (Exception e) {
				Titan.ee("MimicParse", e);
			}
		}
		
		private static final double getDouble(final String data) {
			return getDouble(data, 0.0);
		}
		
		private static final double getDouble(final String data, final double defaultValue) {
			try {
				return Double.parseDouble(data);
			} catch (Throwable e) {
				return defaultValue;
			}
		}
		
		private static final boolean getBoolean(final String data) {
			return getBoolean(data, false);
		}
		
		private static final boolean getBoolean(final String data, final boolean defaultValue) {
			try {
				return Integer.parseInt(data) == 1;
			} catch (Throwable e) {
				return defaultValue;
			}
		}
		
		public String toString() {
			return String.format(formatString, leftDistance, rightDistance, angle, leftPower, rightPower, (isHome) ? 1 : 0, elevatorHeight, intakeTilt, intakeSpeed);
		}
	}
	
	public static class Observer {
		private static FileOutputStream log = null;
		private static boolean homed = false;
		private static boolean saved = true;
		
		public static void prepare(final String fileName) {
			final String fName = String.format(mimicFile, fileName);
			try {
				if(Files.deleteIfExists(new File(fName).toPath())) {
					Titan.e("Deleted previous pathfinding data");
				}
				log = new FileOutputStream(fName);
				saved = false;
				Titan.l("Created new pathfinding file");
			} catch (IOException e) {
				Titan.ee("Mimic", e);
			}
		}
		
		public static void addStep(final Robot robot, final double driveVals[]) {
			try {
                //EDIT THIS
				// final double lDistance = robot.getDrivebase().getLeftDistance();
				// final double rDistance = robot.getDrivebase().getRightDistance();
				// final double angle = robot.getDrivebase().getNavx().getAngle();
				// final double leftPower = driveVals[0];
				// final double rightPower = driveVals[1];
				// boolean home = robot.getTeleop().getLogitech().getRawButton(Titan.LogitechExtreme3D.Button.FIVE);
				// final double elevatorHeight = robot.getElevator().getUpPos();
				// final double intakeTilt = robot.getIntake().getTiltPosition();
				// double intakeSpeed = Constants.INTAKE_STOPPED_SPEED;
				// if(robot.getTeleop().getOperator().getRawButton(Titan.LogitechExtreme3D.Button.TRIGGER)) {
				// 	intakeSpeed = Constants.OUTTAKE_SPEED;
				// } else if(robot.getTeleop().getOperator().getRawButton(Titan.LogitechExtreme3D.Button.TWO)) {
				// 	intakeSpeed = Constants.INTAKE_SPEED;
				// }
				
				// if(home && !homed) {
				// 	robot.getDrivebase().setHome();
				// }
				// if(!saved) log.write(new Stepper(lDistance, rDistance, angle, leftPower, rightPower, home, elevatorHeight, intakeTilt, intakeSpeed).toString().getBytes(StandardCharsets.US_ASCII));
				// homed = home;
			} catch (Exception e) {
				Titan.ee("Mimic", e);
			}
		}
		
		public static void saveMimic() {
			try {
				if(log == null || saved) return;
				Titan.l("Finished observing");
				log.flush();
				log.close();
				saved = true;
				Titan.l("Saved the mimic data");
			} catch (IOException e) {
				Titan.ee("Mimic", e);
			}
		}
	}
	
	public static class Repeater {
		private static FileInputStream log = null;
		private static BufferedReader reader = null;
		private static final ArrayList<Stepper> pathData = new ArrayList<Stepper>();
		
		public static void prepare(final String fileName) {
			final String fName = String.format(mimicFile, fileName);
			try {
				Titan.l("Loading the mimic file");
				if(!Files.exists(new File(fName).toPath())) {
					Titan.e("The requested mimic data was not found");
				}
				
				log = new FileInputStream(fName);
				InputStreamReader iReader = new InputStreamReader(log, StandardCharsets.US_ASCII);
				reader = new BufferedReader(iReader);
				pathData.clear(); //Clear all of the pathData
				
				String line;
				while ((line = reader.readLine()) != null) {
					try {
						pathData.add(new Stepper(line));
					} catch (Exception e) {
						Titan.ee("MimicData", e);
					}
				}
			    
				try {
					reader.close();
				} catch (Exception e) {
					Titan.ee("Failed to close the mimic file", e);
				}
				Titan.l("Loaded the mimic file");
			} catch (IOException e) {
				Titan.ee("Mimic", e);
			}
		}
	
		public static ArrayList<Stepper> getData() {
			return pathData;
		}
	}
}
