package pl.edu.agh.capo.fear.communication;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectOutput;
import java.io.ObjectOutputStream;
import java.util.concurrent.TimeoutException;

import pl.edu.agh.capo.fear.data.LocationInTime;
import pl.edu.agh.capo.fear.data.Trajectory;

import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;
import com.vividsolutions.jts.math.Vector2D;

public class StatePublisher extends StateBase
{

	protected final double trajectoryTimeStep = 0.2;
	protected final int trajectoryStepCount = 8;

	// protected final String rabbitHostIP = "localhost"; // "192.168.2.101"; //
	// "192.168.2.100";
	// protected final String rabbitUser = "panda";
	// protected final String rabbitPass = "panda";

	protected static Channel rabbitmqChannel = null;
	protected final static String rabbitExchangeName = "capoRobotStates";

	public StatePublisher() throws IOException, TimeoutException
	{
		if (rabbitmqChannel == null)
			initRabbit();

	}

	public void close()
	{
		synchronized (StatePublisher.class)
		{
			if (rabbitmqChannel == null)
			{
				try
				{
					rabbitmqChannel.basicCancel("ok");
					rabbitmqChannel.close();
					rabbitmqChannel.getConnection().close();
				}
				catch (IOException e)
				{
				}
				catch (TimeoutException e)
				{
				}
				rabbitmqChannel = null;
			}
		}

	}

	private void initRabbit() throws IOException, TimeoutException
	{

		synchronized (StatePublisher.class)
		{
			ConnectionFactory factory = new ConnectionFactory();
			factory.setHost(rabbitHostIP);
			factory.setUsername(rabbitUser);
			factory.setPassword(rabbitPass);
			Connection connection = factory.newConnection();
			rabbitmqChannel = connection.createChannel();

			rabbitmqChannel.exchangeDeclare(rabbitExchangeName, "fanout");
		}

	}

	public void publishCapoRobotStateAndPlan(Trajectory trajectory)
	{

		// /publish
		synchronized (StatePublisher.class)
		{
			try
			{
				ByteArrayOutputStream bos = new ByteArrayOutputStream();
				ObjectOutput out = new ObjectOutputStream(bos);
				out.writeObject(trajectory);
				rabbitmqChannel.basicPublish(rabbitExchangeName, "", null, bos.toByteArray());
			}
			catch (IOException e)
			{
				System.out.println("ERROR SENDING: " + e.getMessage());
			}
		}

	}

}
