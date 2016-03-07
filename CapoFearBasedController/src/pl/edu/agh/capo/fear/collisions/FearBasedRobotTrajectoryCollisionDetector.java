package pl.edu.agh.capo.fear.collisions;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import com.vividsolutions.jts.algorithm.distance.DiscreteHausdorffDistance.MaxDensifiedByFractionDistanceFilter;
import com.vividsolutions.jts.math.Vector2D;
import com.vividsolutions.jts.math.Vector3D;

import pl.edu.agh.capo.fear.CapoSafeTrajectoryGenerator;
import pl.edu.agh.capo.fear.communication.StateMessageConsumer;
import pl.edu.agh.capo.fear.data.Location;
import pl.edu.agh.capo.fear.data.LocationInTime;
import pl.edu.agh.capo.fear.data.Trajectory;
import pl.edu.agh.capo.robot.CapoRobotMotionModel;

public class FearBasedRobotTrajectoryCollisionDetector implements
		CollisionDetector, StateMessageConsumer
{

	protected HashMap<Integer, Trajectory> robotsTrajectories = new HashMap<Integer, Trajectory>();
	protected int thisRobotId;
	protected double maxObservationDistance;

	public FearBasedRobotTrajectoryCollisionDetector(int thisRobotId, double maxLinearVelocity)
	{
		this.thisRobotId = thisRobotId;
		this.maxObservationDistance = maxLinearVelocity * CapoSafeTrajectoryGenerator.trajectoryStepCount * CapoSafeTrajectoryGenerator.trajectoryTimeStep;
	}

	@Override
	public int getCollidingTrajecotryMinStepNumber(Trajectory trajectory)
	{
		ArrayList<Iterator<LocationInTime>> otherTrajectoriesLocaiontsIterators = getMoreScaryRobotsTrajectories(trajectory);

		int stepId = 1;
		Iterator<LocationInTime> thisTrajectoryLocaiontsIterators = trajectory.getTrajectorySteps().iterator();
		if (!thisTrajectoryLocaiontsIterators.hasNext())
			return -1;
		thisTrajectoryLocaiontsIterators.next();
		while (thisTrajectoryLocaiontsIterators.hasNext())
		{
			Location thisTrajectoryLocation = thisTrajectoryLocaiontsIterators.next().getLocation();
			for (Iterator<LocationInTime> otherTrajectoryLocaiontsIterator : otherTrajectoriesLocaiontsIterators)
			{
				if (otherTrajectoryLocaiontsIterator.hasNext())
				{
					Location otherTrajectoryLocation = otherTrajectoryLocaiontsIterator.next().getLocation();
					if (thisTrajectoryLocation.getDistance(otherTrajectoryLocation) < CapoRobotMotionModel.robotDiameter)
						return stepId;
				}
			}
			stepId++;
		}
		return -1;
	}

	private ArrayList<Iterator<LocationInTime>> getMoreScaryRobotsTrajectories(Trajectory trajectory)
	{
		ArrayList<Iterator<LocationInTime>> moreScaryTrajectoriesLocaiontsIterators = new ArrayList<Iterator<LocationInTime>>();
		synchronized (robotsTrajectories)
		{
			double thisRobotFearFactor = calculateFearFactor(trajectory.getFirstStepLocation(), thisRobotId);
			trajectory.setFearFactor(thisRobotFearFactor);

			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				if (otherTrajectory.getRobotId() == this.thisRobotId)
					// {
					// otherTrajectory.setFearFactor(thisRobotFearFactor);
					continue;
				// }
				if (thisRobotFearFactor > calculateFearFactor(otherTrajectory.getFirstStepLocation(), otherTrajectory.getRobotId()))
					continue;

				Iterator<LocationInTime> otherTrajectoryLocaiontsIterator = otherTrajectory.getTrajectorySteps().iterator();
				if (otherTrajectoryLocaiontsIterator.hasNext())
					otherTrajectoryLocaiontsIterator.next(); // set at second
																// item
				if (otherTrajectoryLocaiontsIterator.hasNext())
					moreScaryTrajectoriesLocaiontsIterators.add(otherTrajectoryLocaiontsIterator);
			}
		}
		return moreScaryTrajectoriesLocaiontsIterators;
	}

	private double calculateFearFactor(Location robotLocation, int robotId)
	{
		double robotFearFactor = calculateFearFactorOryginal(robotLocation, robotId);

		List<Vector3D> GatesList = new ArrayList<Vector3D>();
		GatesList.add(new Vector3D(1.15, 2.5, -Math.PI / 2));

		double gateFearFactor = 0.0;
		gateFearFactor = calculateFearFactorGate(robotLocation, robotId, GatesList);

		return robotFearFactor + gateFearFactor;
	}

	private double calculateFearFactorOryginal(Location robotLocation, int robotId)
	{
		double robotFearFactor = 1.0 + 0.01 * (double) robotId;
		Vector2D robotVersor = new Vector2D(Math.cos(robotLocation.direction), Math.sin(robotLocation.direction));
		for (Trajectory otherTrajectory : robotsTrajectories.values())
		{
			if (otherTrajectory.getRobotId() == robotId)
				continue;
			double angleBetweenRobots = robotVersor.angleTo(new Vector2D(Math.cos(otherTrajectory.getFirstStepLocation().direction), Math.sin(otherTrajectory.getFirstStepLocation().direction)));

			if (Math.abs(angleBetweenRobots) < Math.PI / 2)
			{
				robotFearFactor += ((this.maxObservationDistance - robotLocation.getDistance(otherTrajectory.getFirstStepLocation())) / this.maxObservationDistance) * Math.cos(angleBetweenRobots) * (1.0 + 0.01 * (double) otherTrajectory.getRobotId());
			}
		}

		return robotFearFactor;
	}

	private double calculateFearFactorGate(Location robotLocation, int robotId, List<Vector3D> gateList)
	{
		double robotFearFactor = 1.0 + 0.01 * (double) robotId;
		Vector2D robotVersor = new Vector2D(Math.cos(robotLocation.direction), Math.sin(robotLocation.direction));
		double result = 0.0;

		double temp_psi, temp_G, temp_ff;
		Vector2D temp_robotPosition;
		Vector2D temp_gatePosition;

		for (int l = 0; l < gateList.size(); l++)
		{
			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				temp_robotPosition = new Vector2D(otherTrajectory.getFirstStepLocation().positionX, otherTrajectory.getFirstStepLocation().positionY);
				temp_gatePosition = new Vector2D(gateList.get(l).getX(), gateList.get(l).getY());

				temp_psi = psi(temp_gatePosition, temp_robotPosition);
				temp_G = g(otherTrajectory.getFirstStepLocation().direction, gateList.get(l).getZ());

				if (otherTrajectory.getRobotId() == robotId)
				{
					result += temp_psi * temp_G * robotFearFactor;
					continue;
				}
				else
				{
					double angleBetweenRobots = robotVersor.angleTo(new Vector2D(Math.cos(otherTrajectory.getFirstStepLocation().direction), Math.sin(otherTrajectory.getFirstStepLocation().direction)));

					if (Math.abs(angleBetweenRobots) < Math.PI / 2)
					{
						temp_ff = ((this.maxObservationDistance - robotLocation.getDistance(otherTrajectory.getFirstStepLocation())) / this.maxObservationDistance) * Math.cos(angleBetweenRobots) * (1.0 + 0.01 * (double) otherTrajectory.getRobotId());

						result += temp_psi * temp_G * temp_ff;
					}
					else
						result += temp_psi * temp_G * 0;
				}

			}
		}

		return result;
	}

	private double psi(Vector2D gateCenterPoint, Vector2D robotPosition)
	{
		double distanceGateRobot = getDistance(gateCenterPoint, robotPosition);
		double Rmax = 0.5;
		double result;

		result = (1 - (distanceGateRobot / Rmax)) - (1 - (distanceGateRobot / Rmax)) * heaviside(distanceGateRobot - Rmax);

		return result;
	}

	private double g(double alfa, double gamma)
	{
		return 1 - 2 * (heaviside(gamma - alfa - Math.PI / 2) + heaviside(alfa - gamma - Math.PI / 2));
	}

	public double getDistance(Vector2D pointA, Vector2D pointB)
	{
		return Math.sqrt(Math.pow((pointA.getX() - pointB.getX()), 2.0) + (Math.pow((pointA.getY() - pointB.getY()), 2.0)));
	}

	private double heaviside(double param)
	{
		if (param >= 0)
			return 1;
		else
			return 0;
	}

	protected int messagesSinceLastDatastructureReset = 0;

	@Override
	public void consumeMessage(Trajectory trajectory)
	{
		synchronized (robotsTrajectories)
		{
			if (messagesSinceLastDatastructureReset > 20)
			{
				robotsTrajectories.clear();
				messagesSinceLastDatastructureReset = 0;
			}
			robotsTrajectories.put(trajectory.getRobotId(), trajectory);
		}
	}

}
