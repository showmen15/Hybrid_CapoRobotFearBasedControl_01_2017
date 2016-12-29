package pl.edu.agh.capo.fear;

import java.util.ArrayList;
import java.util.Random;

import com.vividsolutions.jts.math.Vector2D;

import pl.edu.agh.capo.fear.collisions.CollisionDetector;
import pl.edu.agh.capo.fear.collisions.FearBasedRobotTrajectoryCollisionDetector;
import pl.edu.agh.capo.fear.communication.StatePublisher;
import pl.edu.agh.capo.fear.data.LocationInTime;
import pl.edu.agh.capo.fear.data.Trajectory;
import pl.edu.agh.capo.robot.CapoRobotMotionModel;

public class CapoSafeTrajectoryGenerator
{

	public static final double trajectoryTimeStep = 0.2;
	public static final int trajectoryStepCount = 12; // 8 //20

	public static final int alternativeTrajectoriesTested = 100; // 100

	protected ArrayList<CollisionDetector> collisionDetectors = new ArrayList<CollisionDetector>();
	protected Random random = new Random();

	protected double lastBestVelocityLeft = 0, lastBestVelocityRight = 0;

	double timeEsc;

	int tooClose = 0;

	int iDeadlockCount = 0;

	double LastBestAngle;
	Boolean mustRun;

	protected double lastBestVelocity = 0;
	protected double lastBestAngle = 0;

	public CapoSafeTrajectoryGenerator()
	{

	}

	public void addColisionDetector(CollisionDetector collisionDetector)
	{
		collisionDetectors.add(collisionDetector);
	}

	// public Vector2D newDestinations(Vector2D currentLocation, Vector2D
	// targetDestination)
	// {
	// CollisionDetector collisionDetector = collisionDetectors.get(0);
	//
	// if (collisionDetector != null)
	// return collisionDetector.getNextDestination(currentLocation,
	// targetDestination);
	// else
	// return null;
	// }

	public Trajectory getSafeTrajectoryNewNewNew(CapoRobotMotionModel motionModel, Vector2D destination, StatePublisher statePublisher)
	{
		Trajectory trajectory = buildTrajectory(motionModel, destination);

		ArrayList<Trajectory> robotsTrajectory = GetRobotsTrajectoryWithoutCurrent();

		double currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
		trajectory.setFearFactor(currentRobotFF);

		ArrayList<Trajectory> robotCloseToMeBiggerFF = getRobotsWithBiggerFF(trajectory, robotsTrajectory);

		if (robotCloseToMeBiggerFF.size() > 0)
		{
			trajectory = getBestTrajectory(trajectory, motionModel, destination, robotCloseToMeBiggerFF);

		}

		trajectory.setFearFactor(currentRobotFF);
		return trajectory;
	}

	public Trajectory getSafeTrajectoryNewNew(CapoRobotMotionModel motionModel, Vector2D destination, StatePublisher statePublisher)
	{
		Trajectory trajectory = buildTrajectory(motionModel, destination);

		// if (getTrajectoryCost(trajectory, destination) > 0)
		// {
		// iDeadlockCount = 0;
		// return trajectory;
		// }

		ArrayList<Trajectory> robotsTrajectory = GetRobotsTrajectoryWithoutCurrent();
		double currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
		trajectory.setFearFactor(currentRobotFF);

		// ArrayList<Trajectory> robotCloseToMeSmallerFF =
		// getRobotsWithSmallerFF(trajectory, robotsTrajectory);
		//
		// if (robotCloseToMeSmallerFF.size() > 0)
		// iDeadlockCount++;
		// else
		// iDeadlockCount = 0;
		//
		// if (iDeadlockCount >= 5)
		// {
		// motionModel.setVelocity(0, 0);
		// trajectory = buildTrajectory(motionModel, destination);
		// trajectory.setFearFactor(0); // getMinFF(robotCloseToMeSmallerFF) -
		// // 0.1);
		// }
		// else
		// {
		ArrayList<Trajectory> robotCloseToMeBiggerFF = getRobotsWithBiggerFF(trajectory, robotsTrajectory);

		if (robotCloseToMeBiggerFF.size() > 0)
		{
			trajectory = getBestTrajectory(trajectory, motionModel, destination, robotCloseToMeBiggerFF);

			// if (trajectory == null)
			// {
			// motionModel.setVelocity(0, 0);
			// trajectory = buildTrajectory(motionModel, destination);
			// trajectory.setFearFactor(getMinFF(robotCloseToMeSmallerFF) -
			// 0.1);
			// }
			// }
		}

		trajectory.setFearFactor(currentRobotFF);
		return trajectory;
	}

	public Trajectory getSafeTrajectoryNewNewTEST(CapoRobotMotionModel motionModel, Vector2D destination, StatePublisher statePublisher, int robotID)
	{
		Trajectory trajectory = buildTrajectory(motionModel, destination);

		double currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
		trajectory.setFearFactor(currentRobotFF);

		if (getTrajectoryCost(trajectory, destination) > 0)
		{
			lastBestVelocity = 0;

			currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
			trajectory.setFearFactor(currentRobotFF);

			return trajectory;
		}

		double bestAngle = 0;
		double bestVelocity = 0;

		double bestTrajectoryCost = Double.MAX_VALUE;
		double trajectoryCost;
		double bestVelocityLeft = 0, bestVelocityRight = 0;
		Trajectory bestTrajectory = null;

		// // ////// restore last best
		// motionModel.setVelocity(lastBestVelocityLeft,
		// lastBestVelocityRight);
		// trajectory = buildTrajectory(motionModel, destination);
		// trajectoryCost = getTrajectoryCost(trajectory, destination);
		//
		// if (trajectoryCost > 0)
		// {
		// bestVelocityLeft = lastBestVelocityLeft;
		// bestVelocityRight = lastBestVelocityRight;
		// bestTrajectoryCost = trajectoryCost;
		// bestTrajectory = trajectory;
		// }

		double angel = -Math.PI;
		double angleStep = (2 * Math.PI) / alternativeTrajectoriesTested;

		// //////// try find better
		for (int timeout = alternativeTrajectoriesTested; timeout > 0; timeout--)
		{
			motionModel.setVelocity_LinearVelocity_AngularVelocity(motionModel.getMaxLinearVelocity(), angel);

			trajectory = buildTrajectory(motionModel, destination);
			trajectoryCost = getTrajectoryCost(trajectory, destination);

			if (trajectoryCost > 0 && trajectoryCost < bestTrajectoryCost)
			{
				bestVelocityLeft = motionModel.getVelocityLeft();
				bestVelocityRight = motionModel.getVelocityRight();
				bestTrajectoryCost = trajectoryCost;
				bestTrajectory = trajectory;

				bestAngle = angel;
				bestVelocity = motionModel.getMaxLinearVelocity();

			}

			motionModel.setVelocity_LinearVelocity_AngularVelocity(-motionModel.getMaxLinearVelocity(), angel);

			trajectory = buildTrajectory(motionModel, destination);
			trajectoryCost = getTrajectoryCost(trajectory, destination);

			if (trajectoryCost > 0 && trajectoryCost < bestTrajectoryCost)
			{
				bestVelocityLeft = motionModel.getVelocityLeft();
				bestVelocityRight = motionModel.getVelocityRight();
				bestTrajectoryCost = trajectoryCost;
				bestTrajectory = trajectory;

				bestAngle = angel;
				bestVelocity = motionModel.getMaxLinearVelocity();
			}

			angel += angleStep;
		}

		if (bestTrajectoryCost < Double.MAX_VALUE)
		{
			motionModel.setVelocity(bestVelocityLeft, bestVelocityRight);
			lastBestVelocityLeft = bestVelocityLeft;
			lastBestVelocityRight = bestVelocityRight;

			currentRobotFF = CaluclateCurrentRobotTrajectory(bestTrajectory);
			bestTrajectory.setFearFactor(currentRobotFF);

			lastBestAngle = bestAngle;
			lastBestVelocity = bestVelocity;

			currentRobotFF = CaluclateCurrentRobotTrajectory(bestTrajectory);
			bestTrajectory.setFearFactor(currentRobotFF);

			return bestTrajectory;
		}
		else
		{

			motionModel.setVelocity(0, 0);
			trajectory = buildTrajectory(motionModel, destination); // 'stop'

			currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
			trajectory.setFearFactor(currentRobotFF);

			currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
			trajectory.setFearFactor(currentRobotFF);
			return trajectory;
		}
	}

	public Trajectory getSafeTrajectoryNewNew2(CapoRobotMotionModel motionModel, Vector2D destination, StatePublisher statePublisher)
	{
		if (lastBestVelocityLeft != 0)
		{

		}

		double angle = 0.1;
		Trajectory trajectory = buildTrajectory(motionModel, destination);
		double currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
		trajectory.setFearFactor(currentRobotFF);

		double trajectoryCost = getTrajectoryCost(trajectory, destination);

		if (trajectoryCost < 0)
		{
			if (LastBestAngle == 0)
			{
				LastBestAngle = trajectory.getTargetLocation().direction;
			}

			LastBestAngle -= angle;

			motionModel.setVelocity_LinearVelocity_AngularVelocity(-motionModel.getMaxLinearVelocity(), LastBestAngle);
			trajectory = buildTrajectory(motionModel, destination);

			currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
			trajectory.setFearFactor(currentRobotFF);
			mustRun = true;

		}
		else
		{
			mustRun = false;
			LastBestAngle = 0.0;

		}

		/*
		 * // if (getTrajectoryCost(trajectory, destination) > 0) // { //
		 * iDeadlockCount = 0; // return trajectory; // }
		 * 
		 * ArrayList<Trajectory> robotsTrajectory =
		 * GetRobotsTrajectoryWithoutCurrent(); double currentRobotFF =
		 * CaluclateCurrentRobotTrajectory(trajectory);
		 * trajectory.setFearFactor(currentRobotFF);
		 * 
		 * // ArrayList<Trajectory> robotCloseToMeSmallerFF = //
		 * getRobotsWithSmallerFF(trajectory, robotsTrajectory); // // if
		 * (robotCloseToMeSmallerFF.size() > 0) // iDeadlockCount++; // else //
		 * iDeadlockCount = 0; // // if (iDeadlockCount >= 5) // { //
		 * motionModel.setVelocity(0, 0); // trajectory =
		 * buildTrajectory(motionModel, destination); //
		 * trajectory.setFearFactor(0); // getMinFF(robotCloseToMeSmallerFF) -
		 * // // 0.1); // } // else // { ArrayList<Trajectory>
		 * robotCloseToMeBiggerFF = getRobotsWithBiggerFF(trajectory,
		 * robotsTrajectory);
		 * 
		 * if (robotCloseToMeBiggerFF.size() > 0) { trajectory =
		 * getBestTrajectory(trajectory, motionModel, destination,
		 * robotCloseToMeBiggerFF);
		 * 
		 * // if (trajectory == null) // { // motionModel.setVelocity(0, 0); //
		 * trajectory = buildTrajectory(motionModel, destination); //
		 * trajectory.setFearFactor(getMinFF(robotCloseToMeSmallerFF) - // 0.1);
		 * // } // } }
		 * 
		 * trajectory.setFearFactor(currentRobotFF);
		 */
		return trajectory;
	}

	private Trajectory getBestTrajectory(Trajectory trajectoryPrev, CapoRobotMotionModel motionModel, Vector2D destination, ArrayList<Trajectory> robotsTrajectory)
	{
		Trajectory trajectory;

		double bestTrajectoryCost = Double.MAX_VALUE;
		double bestVelocityLeft = 0, bestVelocityRight = 0;
		Trajectory bestTrajectory = null;

		// //////// restore last best
		motionModel.setVelocity(lastBestVelocityLeft, lastBestVelocityRight);
		trajectory = buildTrajectory(motionModel, destination);
		double trajectoryCost = getTrajectoryCost(trajectory, destination);

		if (trajectoryCost > 0)
		{
			bestVelocityLeft = lastBestVelocityLeft;
			bestVelocityRight = lastBestVelocityRight;
			bestTrajectoryCost = trajectoryCost;
			bestTrajectory = trajectory;
		}

		double angle = -Math.PI;
		double step = (2 * Math.PI) / alternativeTrajectoriesTested;

		// //////// try find better
		for (int timeout = alternativeTrajectoriesTested; timeout > 0; timeout--)
		{
			motionModel.setVelocity_LinearVelocity_AngularVelocity(motionModel.getMaxLinearVelocity(), angle);

			trajectory = buildTrajectory(motionModel, destination);
			trajectoryCost = getTrajectoryCost(trajectory, destination);

			if (trajectoryCost > 0 && trajectoryCost < bestTrajectoryCost)
			{
				bestVelocityLeft = motionModel.getVelocityLeft();
				bestVelocityRight = motionModel.getVelocityRight();
				bestTrajectoryCost = trajectoryCost;
				bestTrajectory = trajectory;
			}

			motionModel.setVelocity_LinearVelocity_AngularVelocity(-motionModel.getMaxLinearVelocity(), angle);

			trajectory = buildTrajectory(motionModel, destination);
			trajectoryCost = getTrajectoryCost(trajectory, destination);

			if (trajectoryCost > 0 && trajectoryCost < bestTrajectoryCost)
			{
				bestVelocityLeft = motionModel.getVelocityLeft();
				bestVelocityRight = motionModel.getVelocityRight();
				bestTrajectoryCost = trajectoryCost;
				bestTrajectory = trajectory;
			}

			angle += step;
		}

		if (bestTrajectoryCost < Double.MAX_VALUE)
		{
			motionModel.setVelocity(bestVelocityLeft, bestVelocityRight);
			lastBestVelocityLeft = bestVelocityLeft;
			lastBestVelocityRight = bestVelocityRight;
			return bestTrajectory;
		}
		else
		{
			motionModel.setVelocity(0, 0);
			return buildTrajectory(motionModel, destination); // 'stop'
		}
	}

	private double getTrajectoryCostNew(Trajectory trajectory, Vector2D destination, ArrayList<Trajectory> robotsTrajectory)
	{
		double iPos = 0;
		int index = 0;

		for (LocationInTime locTrajecory : trajectory.getTrajectorySteps())
		{
			for (Trajectory traj2 : robotsTrajectory)
			{
				if (locTrajecory.getLocation().getDistance(traj2.getLocationInTime(index).getLocation()) < 0.3)
					return iPos;
			}

			iPos++;
			index++;
		}

		return iPos;
	}

	private ArrayList<Trajectory> getRobotsWithBiggerFF(Trajectory currentTrajectory, ArrayList<Trajectory> robotsTrajectory)
	{
		ArrayList<Trajectory> colision = new ArrayList<Trajectory>();

		for (Trajectory trajectory : robotsTrajectory)
		{
			if (trajectory.getFearFactor() > currentTrajectory.getFearFactor())
			{
				if (currentTrajectory.getFirstStepLocation().getDistance(trajectory.getFirstStepLocation()) < 0.6)
					colision.add(trajectory);
			}
		}

		return colision;
	}

	private double getMinFF(ArrayList<Trajectory> robotCloseToMeSmallerFF)
	{
		double tempFF = Double.MAX_VALUE;

		for (Trajectory trajectory : robotCloseToMeSmallerFF)
			tempFF = Math.min(tempFF, trajectory.getFearFactor());

		return tempFF;
	}

	private ArrayList<Trajectory> getRobotsWithSmallerFF(Trajectory currentTrajectory, ArrayList<Trajectory> robotsTrajectory)
	{
		ArrayList<Trajectory> colision = new ArrayList<Trajectory>();

		for (Trajectory trajectory : robotsTrajectory)
		{
			if (trajectory.getFearFactor() < currentTrajectory.getFearFactor())
			{
				if (currentTrajectory.getFirstStepLocation().getDistance(trajectory.getFirstStepLocation()) < 0.7)
					colision.add(trajectory);
			}
		}

		return colision;
	}

	private ArrayList<Trajectory> GetRobotsTrajectoryWithoutCurrent()
	{
		for (CollisionDetector collisionDetector : collisionDetectors)
			if (collisionDetector instanceof FearBasedRobotTrajectoryCollisionDetector)
				return ((FearBasedRobotTrajectoryCollisionDetector) collisionDetector).GetRobotsTrajectoryWithoutCurrentRobot();

		return null;
	}

	private ArrayList<Trajectory> GetRobotsSmallerFFTrajectoryWithoutCurrentRobot()
	{
		for (CollisionDetector collisionDetector : collisionDetectors)
			if (collisionDetector instanceof FearBasedRobotTrajectoryCollisionDetector)
				return ((FearBasedRobotTrajectoryCollisionDetector) collisionDetector).GetRobotsSmallerFFTrajectoryWithoutCurrentRobot();

		return null;
	}

	public double CaluclateCurrentRobotTrajectory(Trajectory trajectory)
	{
		for (CollisionDetector collisionDetector : collisionDetectors)
			if (collisionDetector instanceof FearBasedRobotTrajectoryCollisionDetector)
				return ((FearBasedRobotTrajectoryCollisionDetector) collisionDetector).CaluclateCurrentRobotTrajectory(trajectory);

		return 0;
	}

	public Trajectory getSafeTrajectoryNew(CapoRobotMotionModel motionModel, Vector2D destination, StatePublisher statePublisher, int robotID)
	{
		Trajectory trajectory = buildTrajectory(motionModel, destination);

		ArrayList<Trajectory> robotsTajectory = getTrajectoryRobots(trajectory);

		if (robotSmallerWSblocking(robotsTajectory, robotID) > 0)
		{
			motionModel.setVelocity(0, 0);
			return buildTrajectory(motionModel, destination); // 'stop'
		}
		else
		{
			if (robotBiggerWStoClose(robotsTajectory, robotID) > 0)
			{
				motionModel.setVelocity_LinearVelocity_AngularVelocity(
				// both directions:
				1.6 * motionModel.getMaxLinearVelocity() * (random.nextDouble() - 0.5),
				// 0.8 * motionModel.getMaxLinearVelocity() *
				// random.nextDouble(),
				3 * Math.PI * (random.nextDouble() - 0.5));

				// lastBestVelocityLeft = motionModel.getVelocityLeft();
				// lastBestVelocityRight = motionModel.getVelocityRight();

				trajectory = buildTrajectory(motionModel, destination);
				return trajectory;
			}
			else
			{
				return trajectory;
			}
		}
	}

	private int robotSmallerWSblocking(ArrayList<Trajectory> robotsTajectory, int robotID)
	{
		ArrayList<Trajectory> robotsTajectorySmaller = new ArrayList<Trajectory>();
		Trajectory currentRobotTrajecory = robotsTajectory.get(0);

		for (Trajectory trj : robotsTajectory)
		{
			if (trj.getRobotId() == robotID)
				continue;
			else
				if (trj.getFearFactor() < currentRobotTrajecory.getFearFactor())
					robotsTajectorySmaller.add(trj);
		}

		for (Trajectory smaller : robotsTajectorySmaller)
		{
			if (currentRobotTrajecory.getFirstStepLocation().getDistance(smaller.getFirstStepLocation()) < 0.5)
				return 1;
		}

		return -1;
	}

	private int robotBiggerWStoClose(ArrayList<Trajectory> robotsTajectory, int robotID)
	{
		ArrayList<Trajectory> robotsTajectoryBigger = new ArrayList<Trajectory>();
		Trajectory currentRobotTrajecory = robotsTajectory.get(0);

		for (Trajectory trj : robotsTajectory)
		{
			if (trj.getRobotId() == robotID)
				continue;
			else
				if (trj.getFearFactor() > currentRobotTrajecory.getFearFactor())
					robotsTajectoryBigger.add(trj);
		}

		for (Trajectory bigger : robotsTajectoryBigger)
		{
			if (currentRobotTrajecory.getFirstStepLocation().getDistance(bigger.getFirstStepLocation()) < 0.9)
				return 1;
		}

		return -1;
	}

	// ArrayList<Trajectory> robotsColisionTrajectory =
	// getColisionTajecorySmallerWS(robotsTajectory);
	//
	// colisionTrajectorys = getColisionTrajectoryRobots(trajectory);
	//
	// if (colisionTrajectorys.size() > 0)
	// {
	// // mamy potencjalna kolizje z ktoryms z robotow cos trzeba z tym
	// // zrobic
	//
	// // sprawdzenie czy mamy kolizje z robotem o mniejszym WS
	// if (existColisionTrajectoryWithSmallerWSrobot(trajectory,
	// colisionTrajectorys))
	// {
	//
	// }
	// }
	// private ArrayList<Trajectory>
	// getColisionTajecorySmallerWS(ArrayList<Trajectory> trajecorys)
	// {
	//
	// }
	//
	// private Boolean existColisionTrajectoryWithSmallerWSrobot(Trajectory
	// trajectory, ArrayList<Trajectory> colisionTrajectorys)
	// {
	// for (Trajectory trajectory : colisionTrajectorys)
	// {
	// if(trajectory )
	// }
	//
	// return false;
	// }

	/**
	 * 
	 * @param motionModel
	 *            -- velocity can and will be modified if given one causes
	 *            collisions
	 * @param destination
	 * @return Safe (collision-free) trajectory, which brings the robot closest
	 *         to the destination.
	 * 
	 */
	public Trajectory getSafeTrajectory(CapoRobotMotionModel motionModel, Vector2D destination, StatePublisher statePublisher)
	{
		Trajectory trajectory = buildTrajectory(motionModel, destination);

		// ArrayList<Trajectory> robotsTrajectory =
		// GetRobotsTrajectoryWithoutCurrent();
		// double currentRobotFF = CaluclateCurrentRobotTrajectory(trajectory);
		// trajectory.setFearFactor(currentRobotFF);
		//
		// ArrayList<Trajectory> robotCloseToMeSmallerFF =
		// getRobotsWithSmallerFF(trajectory, robotsTrajectory);
		//
		// if (robotCloseToMeSmallerFF.size() > 0)
		// {
		// motionModel.setVelocity(0, 0);
		// trajectory = buildTrajectory(motionModel, destination);
		// trajectory.setFearFactor(getMinFF(robotCloseToMeSmallerFF) - 0.1);
		// return trajectory;
		// }
		// else
		// {
		if (getTrajectoryCost(trajectory, destination) > 0)
			return trajectory;

		double bestTrajectoryCost = Double.MAX_VALUE;
		double bestVelocityLeft = 0, bestVelocityRight = 0;
		Trajectory bestTrajectory = null;

		// //////// restore last best
		motionModel.setVelocity(lastBestVelocityLeft, lastBestVelocityRight);
		trajectory = buildTrajectory(motionModel, destination);
		double trajectoryCost = getTrajectoryCost(trajectory, destination);

		if (trajectoryCost > 0)
		{
			bestVelocityLeft = lastBestVelocityLeft;
			bestVelocityRight = lastBestVelocityRight;
			bestTrajectoryCost = trajectoryCost;
			bestTrajectory = trajectory;
		}

		// //////// try find better
		for (int timeout = alternativeTrajectoriesTested; timeout > 0; timeout--)
		{
			motionModel.setVelocity_LinearVelocity_AngularVelocity(
			// both directions:
			1.6 * motionModel.getMaxLinearVelocity() * (random.nextDouble() - 0.5),
			// 0.8 * motionModel.getMaxLinearVelocity() *
			// random.nextDouble(),
			3 * Math.PI * (random.nextDouble() - 0.5));

			trajectory = buildTrajectory(motionModel, destination);
			trajectoryCost = getTrajectoryCost(trajectory, destination);

			// trajectory.setRobotId(0);
			// statePublisher.publishCapoRobotStateAndPlan(trajectory);
			//
			// System.out.print("Cost:" + trajectoryCost + "\n");

			if (trajectoryCost > 0 && trajectoryCost < bestTrajectoryCost)
			{
				bestVelocityLeft = motionModel.getVelocityLeft();
				bestVelocityRight = motionModel.getVelocityRight();
				bestTrajectoryCost = trajectoryCost;
				bestTrajectory = trajectory;
			}
		}

		if (bestTrajectoryCost < Double.MAX_VALUE)
		{
			motionModel.setVelocity(bestVelocityLeft, bestVelocityRight);
			lastBestVelocityLeft = bestVelocityLeft;
			lastBestVelocityRight = bestVelocityRight;
			return bestTrajectory;
		}
		else
		{
			motionModel.setVelocity(0, 0);
			return buildTrajectory(motionModel, destination); // 'stop'
			// trajectory
		}
		// }
		// }
	}

	private int robotToClose(Trajectory trajectory)
	{
		return getMinWSBlocingRobot(trajectory);
	}

	public Trajectory getMinWSBlocingRobotTrajectory(Trajectory trajectory)
	{
		for (CollisionDetector collisionDetector : collisionDetectors)
			if (collisionDetector instanceof FearBasedRobotTrajectoryCollisionDetector)
				return ((FearBasedRobotTrajectoryCollisionDetector) collisionDetector).getBlocingRobotTrajectory(trajectory);

		return null;
	}

	public ArrayList<Trajectory> getTrajectoryRobots(Trajectory currentTrajectory)
	{
		for (CollisionDetector collisionDetector : collisionDetectors)
			if (collisionDetector instanceof FearBasedRobotTrajectoryCollisionDetector)
				return ((FearBasedRobotTrajectoryCollisionDetector) collisionDetector).getTrajectoryRobots(currentTrajectory);

		return null;
	}

	public Trajectory getSafeTrajectoryTEST(CapoRobotMotionModel motionModel, Vector2D destination, StatePublisher statePublisher)
	{
		Trajectory trajectory = null;
		double bestTrajectoryCost = Double.MAX_VALUE;
		double bestVelocityLeft = 0, bestVelocityRight = 0;
		double trajectoryCost = 0;
		Trajectory bestTrajectory = null;

		// trajectory = buildTrajectory(motionModel, destination);
		//
		// if (getTrajectoryCost(trajectory, destination) > 0) // given is safe
		// return trajectory;

		// //////// restore last best
		// motionModel.setVelocity(lastBestVelocityLeft, lastBestVelocityRight);
		// trajectory = buildTrajectory(motionModel, destination);
		// trajectoryCost = getTrajectoryCost(trajectory, destination);
		//
		// if (trajectoryCost > 0)
		// {
		// bestVelocityLeft = lastBestVelocityLeft;
		// bestVelocityRight = lastBestVelocityRight;
		// bestTrajectoryCost = trajectoryCost;
		// bestTrajectory = trajectory;
		// }

		// //////// try find better
		for (int timeout = alternativeTrajectoriesTested; timeout > 0; timeout--)
		{
			motionModel.setVelocity_LinearVelocity_AngularVelocity(
			// both directions:
			1.6 * motionModel.getMaxLinearVelocity() * (random.nextDouble() - 0.5),
			// 0.8 * motionModel.getMaxLinearVelocity() * random.nextDouble(),
			3 * Math.PI * (random.nextDouble() - 0.5));

			trajectory = buildTrajectory(motionModel, destination);
			trajectoryCost = getTrajectoryCost(trajectory, destination);

			statePublisher.publishCapoRobotStateAndPlan(trajectory);

			System.out.print("Cost:" + trajectoryCost + "\n");

			if (trajectoryCost > 0 && trajectoryCost < bestTrajectoryCost)
			{
				bestVelocityLeft = motionModel.getVelocityLeft();
				bestVelocityRight = motionModel.getVelocityRight();
				bestTrajectoryCost = trajectoryCost;
				bestTrajectory = trajectory;
			}

		}

		if (bestTrajectoryCost < Double.MAX_VALUE)
		{
			motionModel.setVelocity(bestVelocityLeft, bestVelocityRight);
			lastBestVelocityLeft = bestVelocityLeft;

			return bestTrajectory;
		}
		else
		{
			motionModel.setVelocity(0, 0);
			return buildTrajectory(motionModel, destination); // 'stop'
																// trajectory
		}
	}

	public int getMinWSBlocingRobot(Trajectory trajectory)
	{
		for (CollisionDetector collisionDetector : collisionDetectors)
			if (collisionDetector instanceof FearBasedRobotTrajectoryCollisionDetector)
				return ((FearBasedRobotTrajectoryCollisionDetector) collisionDetector).getBlocingRobotWSSmaller(trajectory);

		return -1;
	}

	/**
	 * 
	 * @param trajectory
	 * @return -1 if the trajectory causes collision according to any
	 *         CollisionDetector. Final distance to destination otherwise.
	 */
	private double getTrajectoryCost(Trajectory trajectory, Vector2D destination)
	{
		for (CollisionDetector collisionDetector : collisionDetectors)
			if (collisionDetector.getCollidingTrajecotryMinStepNumber(trajectory) >= 0)
				return -1;

		return trajectory.getLastStepLocation().getPositionVector().distance(destination);
	}

	// private double getTrajectoryCostNewNEW(Trajectory trajectory, Vector2D
	// destination)
	// {
	// for (CollisionDetector collisionDetector : collisionDetectors)
	// if
	// (collisionDetector.getCollidingTrajecotryMinStepNumberNewNew(trajectory)
	// >= 0)
	// return -1;
	//
	// return
	// trajectory.getLastStepLocation().getPositionVector().distance(destination);
	// }

	/**
	 * Just generates steps of the trajectory by given motionModel
	 * 
	 * @param motionModel
	 * @param destination
	 * @return
	 */
	public static Trajectory buildTrajectory(CapoRobotMotionModel motionModel, Vector2D destination)
	{
		Trajectory trajectory = new Trajectory(destination);
		for (int i = 0; i < trajectoryStepCount; i++)
			trajectory.addTrajectoryStep(new LocationInTime(motionModel.getLocationAfterTime(trajectoryTimeStep * i), trajectoryTimeStep * i));
		return trajectory;
	}

	public double getMinCollisionDistance(Trajectory currentTrajectory)
	{
		double minDistance = Double.MAX_VALUE;

		double tempDistance;
		ArrayList<Trajectory> robotsTrajectory = GetRobotsTrajectoryWithoutCurrent();

		for (int i = 0; i < robotsTrajectory.size(); i++)
		{
			tempDistance = robotsTrajectory.get(i).getFirstStepLocation().getDistance(currentTrajectory.getFirstStepLocation());

			minDistance = Math.min(minDistance, tempDistance);
		}

		return minDistance;
	}

	public double GetRobotsSmallerFFTrajectoryWithoutCurrentRobot(Trajectory currentTrajectory)
	{
		double minDistance = Double.MAX_VALUE;

		double tempDistance;
		ArrayList<Trajectory> robotsTrajectory = GetRobotsSmallerFFTrajectoryWithoutCurrentRobot();

		for (int i = 0; i < robotsTrajectory.size(); i++)
		{
			tempDistance = robotsTrajectory.get(i).getFirstStepLocation().getDistance(currentTrajectory.getFirstStepLocation());

			minDistance = Math.min(minDistance, tempDistance);
		}

		return minDistance;
	}
}
