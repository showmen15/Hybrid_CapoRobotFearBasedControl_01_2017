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
		this.maxObservationDistance = 1.0; // maxLinearVelocity *
											// CapoSafeTrajectoryGenerator.trajectoryStepCount
											// *
											// CapoSafeTrajectoryGenerator.trajectoryTimeStep;
	}

	@Override
	public int getCollidingTrajecotryMinStepNumber(Trajectory trajectory)
	{
		ArrayList<Iterator<LocationInTime>> otherTrajectoriesLocaiontsIterators = getMoreScaryRobotsTrajectories(trajectory);// getAllRobotsTrajectories(trajectory);

		int stepId = 1;

		Iterator<LocationInTime> thisTrajectoryLocaiontsIterators = trajectory.getTrajectorySteps().iterator();

		if (!thisTrajectoryLocaiontsIterators.hasNext())
			return -1;

		// thisTrajectoryLocaiontsIterators.next(); // do wykomentowanie

		while (thisTrajectoryLocaiontsIterators.hasNext())
		{
			Location thisTrajectoryLocation = thisTrajectoryLocaiontsIterators.next().getLocation(); // juz
																										// raz
																										// pobralem
																										// naxt

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
					continue;

				if (thisRobotFearFactor > calculateFearFactor(otherTrajectory.getFirstStepLocation(), otherTrajectory.getRobotId()))
					continue;

				Iterator<LocationInTime> otherTrajectoryLocaiontsIterator = otherTrajectory.getTrajectorySteps().iterator();

				// if
				// (trajectory.getFirstStepLocation().getDistance(otherTrajectory.getFirstStepLocation())
				// < CapoRobotMotionModel.robotDiameter)
				// {
				//
				// System.out.println("Robot " + this.thisRobotId + "; X; " +
				// trajectory.getFirstStepLocation().positionX + "; Y; " +
				// trajectory.getFirstStepLocation().positionY +
				//
				// "; Robot " + otherTrajectory.getRobotId() + "; X; " +
				// otherTrajectory.getFirstStepLocation().positionX + "; Y; " +
				// otherTrajectory.getFirstStepLocation().positionY
				//
				// + "; Dis; " +
				// trajectory.getFirstStepLocation().getDistance(otherTrajectory.getFirstStepLocation()));
				// }

				if (otherTrajectoryLocaiontsIterator.hasNext())
					otherTrajectoryLocaiontsIterator.next(); // set at second
																// item
				if (otherTrajectoryLocaiontsIterator.hasNext())
					moreScaryTrajectoriesLocaiontsIterators.add(otherTrajectoryLocaiontsIterator);
			}
		}
		return moreScaryTrajectoriesLocaiontsIterators;
	}

	private ArrayList<Iterator<LocationInTime>> getAllRobotsTrajectories(Trajectory trajectory)
	{
		ArrayList<Iterator<LocationInTime>> moreScaryTrajectoriesLocaiontsIterators = new ArrayList<Iterator<LocationInTime>>();
		synchronized (robotsTrajectories)
		{
			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				if (otherTrajectory.getRobotId() == this.thisRobotId)
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

	private ArrayList<Trajectory> getLowScaryRobotsTrajectories(Trajectory trajectory)
	{
		ArrayList<Trajectory> lowScaryTrajectoriesLocaiontsIterators = new ArrayList<Trajectory>();
		synchronized (robotsTrajectories)
		{
			double thisRobotFearFactor = calculateFearFactor(trajectory.getFirstStepLocation(), thisRobotId);
			trajectory.setFearFactor(thisRobotFearFactor);

			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				// {
				// otherTrajectory.setFearFactor(thisRobotFearFactor);
				// }

				if (otherTrajectory.getRobotId() == this.thisRobotId)
					continue;

				if (thisRobotFearFactor < calculateFearFactor(otherTrajectory.getFirstStepLocation(), otherTrajectory.getRobotId()))
					continue;

				Iterator<LocationInTime> otherTrajectoryLocaiontsIterator = otherTrajectory.getTrajectorySteps().iterator();
				if (otherTrajectoryLocaiontsIterator.hasNext())
					otherTrajectoryLocaiontsIterator.next(); // set at second
																// item
				if (otherTrajectoryLocaiontsIterator.hasNext())
					lowScaryTrajectoriesLocaiontsIterators.add(otherTrajectory);
			}
		}
		return lowScaryTrajectoriesLocaiontsIterators;
	}

	public double calculateFearFactor(Location robotLocation, int robotId)
	{
		// //Org FF
		// double robotFearFactor = calculateFearFactorOryginal(robotLocation,
		// robotId);
		//
		// List<Vector3D> GatesList = new ArrayList<Vector3D>();
		// GatesList.add(new Vector3D(1.15, 2.5, -Math.PI / 2));
		//
		// return robotFearFactor;

		// //FF + WP
		// double robotFearFactor = calculateFearFactorOryginal(robotLocation,
		// robotId);
		//
		// List<Vector3D> GatesList = new ArrayList<Vector3D>();
		// GatesList.add(new Vector3D(1.15, 2.5, -Math.PI / 2));
		//
		// double gateFearFactor = 0.0;
		// gateFearFactor = calculateFearFactorGate(robotLocation, robotId,
		// GatesList);
		//
		// return robotFearFactor + gateFearFactor;

		double robotFearFactor = calculateFearFactorOryginal(robotLocation, robotId);

		List<Vector3D> GatesList = new ArrayList<Vector3D>();
		GatesList.add(new Vector3D(2.5, 2.5, -Math.PI / 2));

		double gateFearFactor = 1.0;
		gateFearFactor = calculateFearFactorGate2(robotLocation, robotId, GatesList);

		return robotFearFactor * gateFearFactor;
	}

	// poprzednia wersja
	/*
	 * private double calculateFearFactorOryginal(Location robotLocation, int
	 * robotId) { double robotFearFactor = 1.0 + 0.01 * (double) robotId;
	 * Vector2D robotVersor = new Vector2D(Math.cos(robotLocation.direction),
	 * Math.sin(robotLocation.direction)); for (Trajectory otherTrajectory :
	 * robotsTrajectories.values()) { if (otherTrajectory.getRobotId() ==
	 * robotId) continue; double angleBetweenRobots = robotVersor.angleTo(new
	 * Vector2D(Math.cos(otherTrajectory.getFirstStepLocation().direction),
	 * Math.sin(otherTrajectory.getFirstStepLocation().direction)));
	 * 
	 * if (Math.abs(angleBetweenRobots) < Math.PI / 2) { robotFearFactor +=
	 * ((this.maxObservationDistance -
	 * robotLocation.getDistance(otherTrajectory.getFirstStepLocation())) /
	 * this.maxObservationDistance) * Math.cos(angleBetweenRobots) * (1.0 + 0.01
	 * * (double) otherTrajectory.getRobotId()); } }
	 * 
	 * return robotFearFactor; }
	 */

	private double calculateFearFactorOryginal(Location robotLocation, int robotId)
	{
		double robotFearFactor = 1.0 + 0.01 * (double) robotId;

		Vector2D robotVersor = new Vector2D(Math.cos(robotLocation.direction), Math.sin(robotLocation.direction));
		synchronized (robotsTrajectories)
		{

			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				if (otherTrajectory.getFirstStepLocation() == null)
					otherTrajectory.getFirstStepLocation();

				if (otherTrajectory.getRobotId() == robotId)
					continue;
				else
					if (robotLocation.getDistance(otherTrajectory.getFirstStepLocation()) > this.maxObservationDistance)
						continue;
					else
					{
						double angleBetweenRobots = robotVersor.angleTo(new Vector2D(Math.cos(otherTrajectory.getFirstStepLocation().direction), Math.sin(otherTrajectory.getFirstStepLocation().direction)));

						if (Math.abs(angleBetweenRobots) < (Math.PI / 2))
						{
							robotFearFactor += ((this.maxObservationDistance - robotLocation.getDistance(otherTrajectory.getFirstStepLocation())) / this.maxObservationDistance) * Math.cos(angleBetweenRobots) * (1.0 + 0.01 * (double) otherTrajectory.getRobotId());
						}
					}
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
			synchronized (robotsTrajectories)
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
		}

		return result;
	}

	private double calculateFearFactorGate2(Location robotLocation, int robotId, List<Vector3D> gateList)
	{
		double p;
		Vector2D temp_robotPosition;
		Vector2D temp_gatePosition;
		double temp_psi, temp_lambda;

		temp_gatePosition = new Vector2D(gateList.get(0).getX(), gateList.get(0).getY());
		temp_robotPosition = new Vector2D(robotLocation.positionX, robotLocation.positionY);

		temp_psi = psi2(temp_robotPosition, temp_gatePosition);
		temp_lambda = lambda(robotLocation.direction, gateList.get(0).getZ());

		p = 1 + temp_psi * temp_lambda;

		return p;
	}

	private double psi2(Vector2D gateCenterPoint, Vector2D robotPosition)
	{
		double distanceGateRobot = getDistance(gateCenterPoint, robotPosition);
		double Rl = 1.0;
		double result;

		if (distanceGateRobot <= Rl)
			result = ((Rl - distanceGateRobot) / Rl);
		else
			result = 0;
		return result;
	}

	private double lambda(double alfa, double gamma)
	{
		Vector2D robotAlfa = new Vector2D(Math.cos(alfa), Math.sin(alfa));
		Vector2D doorGamma = new Vector2D(Math.cos(gamma), Math.sin(gamma));
		double subAlfaGamma = robotAlfa.angleTo(doorGamma);

		double halfPI = Math.PI / 2;

		if ((subAlfaGamma >= (-halfPI)) && (subAlfaGamma <= halfPI))
			return 1;
		else
			return 0;
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

	protected int TTL = 10;

	@Override
	public void consumeMessage(Trajectory trajectory)
	{
		synchronized (robotsTrajectories)
		{
			// for (Trajectory otherTrajectory : robotsTrajectories.values())
			// if (otherTrajectory.isFinished)
			// robotsTrajectories.remove(otherTrajectory);

			if (messagesSinceLastDatastructureReset > 20)
			{
				robotsTrajectories.clear();
				messagesSinceLastDatastructureReset = 0;
			}

			robotsTrajectories.put(trajectory.getRobotId(), trajectory);
		}
	}

	@Override
	public void removeMassage(Trajectory trajectory)
	{
		synchronized (robotsTrajectories)
		{
			robotsTrajectories.remove(trajectory.getRobotId());
		}
	}

	public Trajectory getBlocingRobotTrajectory(Trajectory trajectory)
	{
		double thisRobotFearFactor = calculateFearFactor(trajectory.getFirstStepLocation(), thisRobotId);
		trajectory.setRobotId(this.thisRobotId);
		trajectory.setFearFactor(thisRobotFearFactor);

		Trajectory colisionRobotTrajectory = trajectory;
		double otherRobot = 0.0;

		synchronized (robotsTrajectories)
		{
			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				if (otherTrajectory.getRobotId() == this.thisRobotId)
					continue;

				Location thisTrajectoryLocation = trajectory.getLastStepLocation();
				Location thisOtherTrajectory = otherTrajectory.getFirstStepLocation();

				double dist = thisTrajectoryLocation.getDistance(thisOtherTrajectory);

				if (dist < (CapoRobotMotionModel.robotDiameter + 0.1)) // CapoRobotMotionModel.robotDiameter
				{
					otherRobot = otherTrajectory.getFearFactor(); // calculateFearFactor(otherTrajectory.getFirstStepLocation(),
																	// otherTrajectory.getRobotId());

					System.out.print("otherRobot:" + otherRobot + '\n');

					if (otherRobot < thisRobotFearFactor)
					{
						System.out.print("getBlocingRobotTrajectory:" + this.thisRobotId + '\n');

						thisRobotFearFactor = otherRobot;
						colisionRobotTrajectory = otherTrajectory;
						colisionRobotTrajectory.setFearFactor(otherRobot);
						colisionRobotTrajectory.setRobotId(otherTrajectory.getRobotId());
					}
				}
			}
		}

		// return colisionRobotTrajectory;

		if (this.thisRobotId != colisionRobotTrajectory.getRobotId())
			return colisionRobotTrajectory;
		else
			return null;
	}

	public int getBlocingRobotWSSmaller(Trajectory trajectory)
	{
		ArrayList<Trajectory> otherTrajectoriesLocaiontsIterators = getLowScaryRobotsTrajectories(trajectory);

		for (Trajectory trajectory2 : otherTrajectoriesLocaiontsIterators)
		{
			if (trajectory.getFirstStepLocation().getDistance(trajectory2.getFirstStepLocation()) < 0.7)
				return 1;
		}

		return -1;
	}

	public ArrayList<Trajectory> getTrajectoryRobots(Trajectory currentTrajectory)
	{
		Location robotPosition = currentTrajectory.getFirstStepLocation(); // pobieramy
																			// aktualna
																			// pozycje
																			// robota
		double observationRange = robotPosition.getDistance(currentTrajectory.getLastStepLocation()); // wyznaczenie
																										// zasiegu
																										// dzialania
																										// robota

		ArrayList<Trajectory> colisionRobots = new ArrayList<Trajectory>();

		currentTrajectory.setFearFactor(calculateFearFactor(currentTrajectory.getFirstStepLocation(), thisRobotId));

		colisionRobots.add(currentTrajectory);

		synchronized (robotsTrajectories)
		{
			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				otherTrajectory.setFearFactor(calculateFearFactor(otherTrajectory.getFirstStepLocation(), thisRobotId));

				if (otherTrajectory.getRobotId() == this.thisRobotId)
					continue;
				else
				{
					for (LocationInTime otherLocation : otherTrajectory.getTrajectorySteps())
					{
						if (observationRange > robotPosition.getDistance(otherLocation.getLocation()))
						{
							colisionRobots.add(otherTrajectory);
							break;
						}
					}
				}
			}
		}

		return colisionRobots;
	}

	public ArrayList<Trajectory> GetRobotsTrajectoryWithoutCurrentRobot()
	{
		ArrayList<Trajectory> traj = new ArrayList<Trajectory>();

		synchronized (robotsTrajectories)
		{
			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				if (otherTrajectory.getRobotId() == this.thisRobotId)
					continue;
				else
				{
					otherTrajectory.setFearFactor(calculateFearFactor(otherTrajectory.getFirstStepLocation(), otherTrajectory.getRobotId()));

					traj.add(otherTrajectory);
				}
			}
		}
		return traj;
	}

	public ArrayList<Trajectory> GetRobotsSmallerFFTrajectoryWithoutCurrentRobot()
	{
		ArrayList<Trajectory> traj = new ArrayList<Trajectory>();
		Trajectory currentRoboTrajectory = getCurrentRobotTrajectory();

		currentRoboTrajectory.setFearFactor(calculateFearFactor(currentRoboTrajectory.getFirstStepLocation(), currentRoboTrajectory.getRobotId()));

		synchronized (robotsTrajectories)
		{
			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				if (otherTrajectory.getRobotId() == this.thisRobotId)
					continue;
				else
				{
					otherTrajectory.setFearFactor(calculateFearFactor(otherTrajectory.getFirstStepLocation(), otherTrajectory.getRobotId()));

					if (otherTrajectory.getFearFactor() <= currentRoboTrajectory.getFearFactor())
						traj.add(otherTrajectory);
				}
			}
		}
		return traj;
	}

	private Trajectory getCurrentRobotTrajectory()
	{
		synchronized (robotsTrajectories)
		{
			for (Trajectory otherTrajectory : robotsTrajectories.values())
			{
				if (otherTrajectory.getRobotId() == this.thisRobotId)
					return otherTrajectory;
			}
		}
		return null;
	}

	public double CaluclateCurrentRobotTrajectory(Trajectory trajectory)
	{
		return calculateFearFactor(trajectory.getFirstStepLocation(), this.thisRobotId);
	}

}
