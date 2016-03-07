package pl.edu.agh.capo.fear;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;
import java.util.Random;
import java.util.concurrent.TimeoutException;

import pl.edu.agh.amber.common.AmberClient;
import pl.edu.agh.amber.hokuyo.HokuyoProxy;
import pl.edu.agh.amber.hokuyo.Scan;
import pl.edu.agh.amber.roboclaw.RoboclawProxy;
import pl.edu.agh.capo.fear.collisions.FearBasedRobotTrajectoryCollisionDetector;
import pl.edu.agh.capo.fear.collisions.MazeCollisionDetector;
import pl.edu.agh.capo.fear.collisions.RobotTrajectoryCollisionDetector;
import pl.edu.agh.capo.fear.communication.StateCollector;
import pl.edu.agh.capo.fear.communication.StatePublisher;
import pl.edu.agh.capo.fear.data.Location;
import pl.edu.agh.capo.fear.data.LocationInTime;
import pl.edu.agh.capo.fear.data.Trajectory;
import pl.edu.agh.capo.maze.Gate;
import pl.edu.agh.capo.maze.GateNode;
import pl.edu.agh.capo.maze.MazeMap;
import pl.edu.agh.capo.maze.Node;
import pl.edu.agh.capo.maze.NodeNode;
import pl.edu.agh.capo.maze.Room;
import pl.edu.agh.capo.maze.SpaceNode;
import pl.edu.agh.capo.maze.helper.MazeHelper;
import pl.edu.agh.capo.robot.CapoRobotMock;
import pl.edu.agh.capo.robot.CapoRobotMotionModel;
import pl.edu.agh.capo.maze.helper.Dijkstra;

import com.vividsolutions.jts.math.Vector2D;

public class CapoFearBasedControllerMock implements Runnable
{

	protected AmberClient client;
	protected RoboclawProxy roboclawProxy;
	protected HokuyoProxy hokuyoProxy;
	protected CapoRobotMock capoRobotMock;

	protected Thread monitorThread;

	protected CapoRobotMotionModel capoRobotMotionModel;
	protected StatePublisher statePublisher;

	protected CapoSafeTrajectoryGenerator capoSafeTrajectoryGenerator;

	protected Vector2D targetDestination = null, destination = null,
			nextDestination = null;
	protected int robotId;
	protected double InitX;
	protected double InitY;
	protected double InitAngle;
	protected double MaxLinearVelocity;
	protected int LoopNumber;

	protected Dijkstra graph;
	protected List<Room> Rooms;
	protected List<SpaceNode> spaceNodes;
	// protected List<GateNode> gateNodes;

	protected List<Node> nodes;

	protected List<Vector2D> targetList = null;

	public void SetMonitorThread(Thread monitorThread)
	{
		this.monitorThread = monitorThread;
	}

	public CapoFearBasedControllerMock(int robotId, double maxLinearVelocity, String mazeRosonFilename, double initX, double initY, double initAngle) throws Exception
	{
		this.robotId = robotId;

		// this.client = new AmberClient("192.168.2."+(200+robotId), 26233);
		// this.roboclawProxy = new RoboclawProxy(this.client, 0);
		// this.hokuyoProxy = new HokuyoProxy(this.client, 0);

		InitX = initX;
		InitY = initY;
		InitAngle = initAngle;
		MaxLinearVelocity = maxLinearVelocity;
		LoopNumber = 0;

		this.capoRobotMock = new CapoRobotMock(maxLinearVelocity, initX, initY, initAngle);

		/*
		 * if (robotId == 1) this.capoRobotMock = new
		 * CapoRobotMock(maxLinearVelocity, 0.5, 4, -Math.PI / 2); else
		 * this.capoRobotMock = new CapoRobotMock(maxLinearVelocity, 1.5, 4,
		 * -Math.PI / 2);
		 */

		// this.capoRobotMock = new CapoRobotMock(maxLinearVelocity, robotId, 4,
		// -Math.PI / 2);
		// this.capoRobotMock = new CapoRobotMock(maxLinearVelocity);

		this.statePublisher = new StatePublisher();

		this.capoRobotMotionModel = new CapoRobotMotionModel(maxLinearVelocity);

		this.capoSafeTrajectoryGenerator = new CapoSafeTrajectoryGenerator();
		this.capoSafeTrajectoryGenerator.addColisionDetector(new MazeCollisionDetector(mazeRosonFilename));
		// RobotTrajectoryCollisionDetector robotTrajectoryCollisionDetector =
		// new RobotTrajectoryCollisionDetector(robotId);
		FearBasedRobotTrajectoryCollisionDetector robotTrajectoryCollisionDetector = new FearBasedRobotTrajectoryCollisionDetector(robotId, maxLinearVelocity);
		this.capoSafeTrajectoryGenerator.addColisionDetector(robotTrajectoryCollisionDetector);

		StateCollector stateCollector = new StateCollector();
		stateCollector.setConsumer(robotTrajectoryCollisionDetector);

		Rooms = MazeHelper.buildRooms(MazeMap.loadMazeFromFile(mazeRosonFilename));
		graph = initGraph(MazeMap.loadMazeFromFile(mazeRosonFilename).getNodeNodes());

		spaceNodes = MazeMap.loadMazeFromFile(mazeRosonFilename).getSpaceNodes();
		// gateNodes =
		// MazeMap.loadMazeFromFile(mazeRosonFilename).getGateNodes();

		nodes = MazeMap.loadMazeFromFile(mazeRosonFilename).getNodes();
	}

	private Dijkstra initGraph(List<NodeNode> nodeNodes)
	{
		Dijkstra temp = new Dijkstra();

		for (int i = 0; i < nodeNodes.size(); i++)
		{
			NodeNode item = nodeNodes.get(i);

			temp.add_vertex(item.getNodeFromId(), item.getNodeToId(), item.getCost());
		}

		return temp;
	}

	private List<Vector2D> getSubTargets(Vector2D currentRobotLocation, Vector2D targetDestination)
	{
		List<Vector2D> result = new ArrayList<Vector2D>();

		String spaceRobot = getSpaceFormPostion(currentRobotLocation);
		String spaceTarget = getSpaceFormPostion(targetDestination);

		List<String> shortPath = graph.shortest_path(spaceTarget, spaceRobot);

		for (String node : shortPath)
		{
			if (isGateNode(node))
				result.add(getNodePosition(node));
		}

		result.add(targetDestination);
		return result;
	}

	private String getSpaceFormPostion(Vector2D position)
	{
		String result = "";
		Room tempRoom = getRoomFromPosition(Rooms, position);
		String spaceName = tempRoom.getSpaceId();

		for (SpaceNode spaceItem : spaceNodes)
		{
			if (spaceName.equals(spaceItem.getSpaceId()))
			{
				if (isSpaceNode(spaceItem.getNodeId()))
				{
					result = spaceItem.getNodeId();
					return result;
				}
			}
		}

		return result;
	}

	private Boolean isGateNode(String sGateNode)
	{
		String gateNode = "gateNode";

		for (Node nodeItem : nodes)
		{
			if (sGateNode.equals(nodeItem.getId()) && gateNode.equals(nodeItem.getKind()))
				return true;
		}

		return false;
	}

	private Room getRoomFromPosition(List<Room> rooms, Vector2D position)
	{
		Room temp = null;
		for (int i = 0; i < rooms.size(); i++)
		{
			temp = rooms.get(i);

			if ((temp.getMinX() <= position.getX()) && (temp.getMaxX() >= position.getX()) && (temp.getMinY() <= position.getY()) && (temp.getMaxY() >= position.getY()))
				return temp;
		}
		return null;
	}

	private Boolean isSpaceNode(String sSpaceNode)
	{
		String spaceNode = "spaceNode";

		for (Node nodeItem : nodes)
		{
			if (sSpaceNode.equals(nodeItem.getId()) && spaceNode.equals(nodeItem.getKind()))
				return true;
		}

		return false;
	}

	private Vector2D getNodePosition(String nodeName)
	{
		for (Node nodeItem : nodes)
		{
			if (nodeName.equals(nodeItem.getId()))
				return new Vector2D(nodeItem.getPosition().getX(), nodeItem.getPosition().getY());
		}
		return null;
	}

	private Vector2D getCurrentDestination(Vector2D currentRobotLocation, Vector2D targetDestination)
	{
		if (targetList == null)
			targetList = getSubTargets(currentRobotLocation, targetDestination);

		return targetList.get(0);
	}

	private void removeReachedTarget()
	{
		if (targetList.size() > 0)
			targetList.remove(0);
	}

	// private String getSpaceNode(String spaceName)
	// {
	// List<SpaceNode> temp = new ArrayList<SpaceNode>()
	// }

	/*
	 * public void setDestination(Vector2D destination) { this.destination =
	 * destination; }
	 */

	public void setLoopDestinations(Vector2D destination, Vector2D nextDestination)
	{
		this.targetDestination = destination;
		// this.nextDestination = nextDestination;
	}

	protected boolean isRun = true;

	public void Stop()
	{
		this.isRun = false;
		capoRobotMock.SetRoboClawVelocity(0.0D, 0.0D);
		this.statePublisher.close();
	}

	protected boolean isPaused = false;

	public void togglePaused()
	{
		this.isPaused = !this.isPaused;
	}

	public void run()
	{
		try
		{
			while (this.isRun)
			{
				LoopNumber++;
				//System.out.println("Wynik " + LoopNumber);
				this.capoRobotMotionModel.setLocation(capoRobotMock.GetRobotLocation());

				destination = getCurrentDestination(this.capoRobotMotionModel.getLocation().getPositionVector(), this.targetDestination);

				double destinationDistance = destination.distance(this.capoRobotMotionModel.getLocation().getPositionVector());

				if (destinationDistance < CapoRobotMotionModel.wheelsHalfDistance)
				{
					if (destination == targetDestination)
					{
						this.capoRobotMotionModel.setVelocity(0, 0);

						saveToFile();
						System.exit(1);
					}
					else
					{
						removeReachedTarget(); // usuwamy z listy osiagniety cel
												// i pobieramy kolejny
						destination = getCurrentDestination(this.capoRobotMotionModel.getLocation().getPositionVector(), this.targetDestination);
						destinationDistance = targetDestination.distance(this.capoRobotMotionModel.getLocation().getPositionVector());
					}
				}

				if (this.isPaused)
				{
					this.capoRobotMotionModel.setVelocity(0, 0);
				}
				else
				{
					Vector2D robotVersor = capoRobotMotionModel.getVersor();
					Vector2D targetVector = destination.subtract(this.capoRobotMotionModel.getLocation().getPositionVector());
					double angleToTarget = robotVersor.angleTo(targetVector);

					if (destination == targetDestination)
						this.capoRobotMotionModel.setVelocity_LinearVelocity_AngularVelocity(Math.cos(angleToTarget / 2) * Math.min(1, destinationDistance) * this.capoRobotMotionModel.getMaxLinearVelocity() / 2, 2 * angleToTarget);
					else
						this.capoRobotMotionModel.setVelocity_LinearVelocity_AngularVelocity(Math.cos(angleToTarget / 2) * this.capoRobotMotionModel.getMaxLinearVelocity() / 2, 2 * angleToTarget);
					
				}
				Trajectory trajectory = this.capoSafeTrajectoryGenerator.getSafeTrajectory(this.capoRobotMotionModel, destination);
				trajectory.setRobotId(robotId);

				// System.out.println("Run loop: location=(" +
				// this.capoRobotMotionModel.getLocation().positionX + "; " +
				// this.capoRobotMotionModel.getLocation().positionY + "; " +
				// this.capoRobotMotionModel.getLocation().direction + ")" +
				// ";  velocity=(" + this.capoRobotMotionModel.getVelocityLeft()
				// +
				// "; " + this.capoRobotMotionModel.getVelocityRight() + ") ");

				this.monitorThread.interrupt();
				
				//System.out.println("Left: " + this.capoRobotMotionModel.getVelocityLeft() + "Right: " +  this.capoRobotMotionModel.getVelocityRight());
				
				capoRobotMock.SetRoboClawVelocity(this.capoRobotMotionModel.getVelocityLeft(), this.capoRobotMotionModel.getVelocityRight());

				statePublisher.publishCapoRobotStateAndPlan(trajectory);

			}
		}
		catch (Exception ex)
		{		
			LoopNumber = -1;
			saveToFile();
			System.exit(0);
		}
	}

	void reduceSpeedDueToSensorRedingTimeout()
	{
		System.out.print("-> reduceSpeedDueToSensorRedingTimeou  \n");
		capoRobotMock.SetRoboClawVelocity(this.capoRobotMotionModel.getVelocityLeft() / 2.0D, this.capoRobotMotionModel.getVelocityRight() / 2.0D);
	}

	private void saveToFile()
	{
		String sPath = "D:\\Desktop\\result.txt";
		String sResult = "";

		sResult += robotId + ";" + InitX + ";" + InitY + ";" + InitAngle + ";" + MaxLinearVelocity + ";" + destination.getX() + ";" + destination.getY() + ";" + LoopNumber + ";\n";

		try
		{
			Files.write(Paths.get(sPath), sResult.getBytes(), StandardOpenOption.APPEND);
		}
		catch (IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	// private Vector2D getCurrentDestination(Vector2D currentDestination,
	// Vector2D robotVector, List<Room> rooms, Vector2D targetDestination)
	// {
	// if (currentDestination != null)
	// {
	// double subDestinationDistance = currentDestination.distance(robotVector);
	//
	// if (subDestinationDistance < CapoRobotMotionModel.wheelsHalfDistance)
	// {
	// // pobranie kolejnego punktu do ktorego musimy dojechac
	// // return getSubDestination(null, currentDestination, rooms,
	// // targetDestination);
	//
	// Room robotRoom = getRoomFromPosition(rooms, robotVector);
	// Room targetRoom = getRoomFromPosition(rooms, targetDestination);
	//
	// }
	//
	// return currentDestination;
	// }
	// else
	// return getSubDestination(null, robotVector, rooms, targetDestination);
	// }

	private Vector2D getSubDestination(Vector2D currentDestination, Vector2D robotVector, List<Room> rooms, Vector2D targetDestination)
	{
		Room robotRoom = getRoomFromPosition(rooms, robotVector);
		Room targetRoom = getRoomFromPosition(rooms, targetDestination);

		if (robotRoom.getSpaceId() == targetRoom.getSpaceId())
			return targetDestination;
		else
			return getClosestGate(robotRoom, targetDestination);
	}

	private Vector2D getClosestGate(Room currentRobotRoom, Vector2D targetDestination)
	{
		Vector2D result;
		double nextDistance;

		Gate tempGate = currentRobotRoom.getGates().get(0);
		Vector2D tempCenterGate = new Vector2D(tempGate.getCenter().getX(), tempGate.getCenter().getY());

		double distance = targetDestination.distance(tempCenterGate);
		result = tempCenterGate;

		for (int i = 1; i < currentRobotRoom.getGates().size(); i++)
		{
			tempGate = currentRobotRoom.getGates().get(i);
			tempCenterGate = new Vector2D(tempGate.getCenter().getX(), tempGate.getCenter().getY());

			nextDistance = targetDestination.distance(tempCenterGate);

			if (distance < nextDistance)
			{
				distance = nextDistance;
				result = tempCenterGate;
			}
		}

		return result;
	}

	// protected synchronized void SetRoboClawVelocity(double vLeft, double
	// vRight)
	// {
	// if ( Math.abs(this.currentVelocityLeft - vRight) <
	// Math.abs(this.currentVelocityLeft / 8)
	// &&
	// Math.abs(this.currentVelocityRight - vRight) <
	// Math.abs(this.currentVelocityRight / 8) )
	// {
	// System.out.println("At: " + System.currentTimeMillis() +
	// " SKIPPING SMALL CHANGE ");
	// return;
	// }
	//
	//
	// this.currentVelocityLeft = vLeft;
	// this.currentVelocityRight = vRight;
	// this.capoRobotMotionModel.setVelocity(vLeft, vRight);
	// System.out.println("At: " + System.currentTimeMillis() +
	// " set velocity from tread " + Thread.currentThread().getId() + ": left="
	// + vLeft + "; right=" + vRight);
	// try
	// {
	// this.roboclawProxy.sendMotorsCommand((int)(vLeft * 1000.0D), (int)(vRight
	// * 1000.0D), (int)(vLeft * 1000.0D), (int)(vRight * 1000.0D));
	// }
	// catch (Exception e)
	// {
	// System.out.println("Exception in roboclawProxy.sendMotorsCommand: " +
	// e.getMessage());
	// }
	// }
}
