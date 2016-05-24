package pl.edu.agh.capo.fear.communication;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.ObjectInput;
import java.io.ObjectInputStream;
import java.util.concurrent.TimeoutException;

import pl.edu.agh.capo.fear.data.Trajectory;

import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Consumer;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

public class StateCollector extends StateBase
{

	protected static Channel channel = null;
	private StateMessageConsumer stateMessageConsumer;

	// protected final String rabbitHostIP = "192.168.2.101";// "192.168.2.100";
	// protected final String rabbitUser = "panda";
	// protected final String rabbitPass = "panda";

	public StateCollector()
	{
		if (channel == null)
			initializeCommection();
	}

	public void setConsumer(StateMessageConsumer stateMessageConsumer)
	{
		this.stateMessageConsumer = stateMessageConsumer;
	}

	protected void initializeCommection()
	{
		synchronized (StateCollector.class)
		{

			ConnectionFactory factory = new ConnectionFactory();
			factory.setHost(rabbitHostIP);
			factory.setUsername(rabbitUser);
			factory.setPassword(rabbitPass);
			try
			{
				Connection connection = factory.newConnection();
				channel = connection.createChannel();

				channel.exchangeDeclare(channelName, "fanout");
				String queueName = channel.queueDeclare().getQueue();
				channel.queueBind(queueName, channelName, "");

				System.out.println(" [RABBIT] Waiting for messages");

				Consumer consumer = new DefaultConsumer(channel)
				{
					@Override
					public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
							throws IOException
					{
						if (stateMessageConsumer != null)
						{
							ByteArrayInputStream bis = new ByteArrayInputStream(body);
							ObjectInput in = null;
							in = new ObjectInputStream(bis);
							try
							{
								Trajectory t = (Trajectory) in.readObject();
								stateMessageConsumer.consumeMessage(t);
							}
							catch (ClassNotFoundException e)
							{
								System.out.println(" [RABBIT] Error parsing message");
							}
						}
					}
				};
				channel.basicConsume(queueName, true, consumer);
			}
			catch (IOException e)
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			catch (TimeoutException e)
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}
}
