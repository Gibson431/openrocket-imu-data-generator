package io.github.gibson431.openrocketimudatagenerator;

import java.io.File;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.util.regex.Pattern;

import javax.swing.JOptionPane;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import info.openrocket.core.simulation.FlightDataType;
import info.openrocket.core.simulation.FlightEvent;
import info.openrocket.core.simulation.SimulationConditions;
import info.openrocket.core.simulation.SimulationStatus;
import info.openrocket.core.simulation.exception.SimulationException;
import info.openrocket.core.simulation.extension.AbstractSimulationExtension;
import info.openrocket.core.simulation.listeners.AbstractSimulationListener;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.CoordinateIF;
import info.openrocket.core.util.Quaternion;

/**
 * Logs mock IMU sensor data to a CSV file: a body-frame accelerometer,
 * a body-frame rate gyro, and absolute launch-frame position, for use as
 * synthetic sensor input when testing an INS algorithm. The true orientation
 * quaternion is also logged as ground truth, for scoring an INS's attitude
 * estimate - it is not itself a sensor reading.
 * <p>
 * The accelerometer channel is reconstructed as specific force rather than
 * raw kinematic acceleration: OpenRocket's internal body-frame acceleration
 * (FlightDataType.TYPE_ACCELERATION_BODY*) has gravity's contribution to the
 * trajectory already netted out, so it reads ~0 at rest on the pad. A real
 * accelerometer reads ~9.81 m/s^2 in that situation, so gravity is added
 * back in here, rotated into the body frame at the rocket's current
 * orientation, before it is logged.
 */
public class ImuDataGenerator extends AbstractSimulationExtension {

	private static final Logger log = LoggerFactory.getLogger(ImuDataGenerator.class);

	public static final String RUN_DIRECTORY_FORMAT = "%s-%03d";
	public static final String DEFAULT_SIMULATION_NAME = "simulation";
	public static final String DATA_FILENAME = "data.csv";
	public static final String EVENTS_FILENAME = "events.csv";

	private static final Pattern INVALID_FILENAME_CHARS = Pattern.compile("[\\\\/:*?\"<>|]");

	private PrintStream dataOutput = null;
	private PrintStream eventsOutput = null;

	@Override
	public String getName() {
		return "IMU Data Generator (CSV)";
	}

	@Override
	public String getDescription() {
		return "Log mock IMU output (body-frame accelerometer + rate gyro), absolute launch-frame "
				+ "position, and the true orientation quaternion (ground truth) to a CSV file, for use as "
				+ "synthetic sensor input when testing INS algorithms.";
	}

	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(new Listener());
	}

	private class Listener extends AbstractSimulationListener {

		@Override
		public void startSimulation(SimulationStatus status) throws SimulationException {
			if (dataOutput != null || eventsOutput != null) {
				log.warn("WARNING: Ending previous IMU data generator log (SIMULATION_END not encountered).");
				closeOutputs();
			}

			String simulationName = status.getSimulationConditions().getSimulation().getName();
			String sanitizedName = INVALID_FILENAME_CHARS.matcher(simulationName).replaceAll("_").trim();
			if (sanitizedName.isEmpty()) {
				sanitizedName = DEFAULT_SIMULATION_NAME;
			}

			File runDirectory = null;
			try {
				int n = 1;
				do {
					runDirectory = new File(String.format(RUN_DIRECTORY_FORMAT, sanitizedName, n));
					n++;
				} while (!runDirectory.mkdir());
				log.info("IMU data generator run directory is " + runDirectory.getName());

				dataOutput = new PrintStream(new File(runDirectory, DATA_FILENAME), StandardCharsets.UTF_8);
				dataOutput.println("# time_s,pos_x_m,pos_y_m,pos_z_m,"
						+ "accel_body_x_mss,accel_body_y_mss,accel_body_z_mss,"
						+ "gyro_roll_rads,gyro_pitch_rads,gyro_yaw_rads,"
						+ "truth_quat_w,truth_quat_x,truth_quat_y,truth_quat_z");
				dataOutput.flush();

				eventsOutput = new PrintStream(new File(runDirectory, EVENTS_FILENAME), StandardCharsets.UTF_8);
				eventsOutput.println("# time_s,event");
				eventsOutput.flush();
			} catch (Exception e) {
				log.error("ERROR OPENING FILE: " + e);
				JOptionPane.showMessageDialog(null,
						"Error opening file:\n" + e.getMessage(),
						"Error opening file in: " + runDirectory,
						JOptionPane.ERROR_MESSAGE);
			}
		}

		@Override
		public boolean handleFlightEvent(SimulationStatus status, FlightEvent event) throws SimulationException {
			if (eventsOutput != null && event.getType() != FlightEvent.Type.ALTITUDE) {
				eventsOutput.printf("%f,%s%n", event.getTime(), event.getType());
				eventsOutput.flush();
			}
			return true;
		}

		@Override
		public void postStep(SimulationStatus status) throws SimulationException {
			if (dataOutput == null) {
				return;
			}

			CoordinateIF position = status.getRocketPosition();

			double gravity = status.getSimulationConditions().getGravityModel()
					.getGravity(status.getRocketWorldPosition());

			CoordinateIF kinematicAccelBody = new Coordinate(
					status.getFlightDataBranch().getLast(FlightDataType.TYPE_ACCELERATION_BODYX),
					status.getFlightDataBranch().getLast(FlightDataType.TYPE_ACCELERATION_BODYY),
					status.getFlightDataBranch().getLast(FlightDataType.TYPE_ACCELERATION_BODYZ));

			Quaternion orientation = status.getRocketOrientationQuaternion();
			CoordinateIF gravityCorrectionBody = orientation.invRotate(new Coordinate(0, 0, gravity));
			CoordinateIF specificForceBody = kinematicAccelBody.add(gravityCorrectionBody);

			double rollRate = status.getFlightDataBranch().getLast(FlightDataType.TYPE_ROLL_RATE);
			double pitchRate = status.getFlightDataBranch().getLast(FlightDataType.TYPE_PITCH_RATE);
			double yawRate = status.getFlightDataBranch().getLast(FlightDataType.TYPE_YAW_RATE);

			dataOutput.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f%n",
					status.getSimulationTime(),
					position.getX(), position.getY(), position.getZ(),
					specificForceBody.getX(), specificForceBody.getY(), specificForceBody.getZ(),
					rollRate, pitchRate, yawRate,
					orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ());
			dataOutput.flush();
		}

		@Override
		public void endSimulation(SimulationStatus status, SimulationException exception) {
			if (dataOutput != null || eventsOutput != null) {
				log.info("Closing IMU data generator log files");
				closeOutputs();
			}
		}

		private void closeOutputs() {
			if (dataOutput != null) {
				dataOutput.close();
				dataOutput = null;
			}
			if (eventsOutput != null) {
				eventsOutput.close();
				eventsOutput = null;
			}
		}
	}
}
