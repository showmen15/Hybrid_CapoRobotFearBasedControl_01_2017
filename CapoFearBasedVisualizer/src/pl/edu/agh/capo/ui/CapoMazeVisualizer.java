package pl.edu.agh.capo.ui;

import java.awt.Dimension;
import java.awt.Panel;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;

import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JSplitPane;
import javax.swing.WindowConstants;

import org.apache.log4j.Logger;

import pl.edu.agh.capo.fear.communication.StateMessageConsumer;
import pl.edu.agh.capo.fear.data.Trajectory;
import pl.edu.agh.capo.maze.MazeMap;

public class CapoMazeVisualizer extends JFrame implements StateMessageConsumer
{

	private static final Dimension FRAME_SIZE = new Dimension(800, 660);
	private static final int SPLIT_DIVIDER_LOCATION = 600;

	private final Logger logger = Logger.getLogger(CapoMazeVisualizer.class);

	private static final CapoMazeVisualizer instance = new CapoMazeVisualizer();

	private MazePanel mazePanel;

	private CapoMazeVisualizer()
	{
		super("CAPO maze editor");
	}

	public static CapoMazeVisualizer getInstance()
	{
		return instance;
	}

	public void open(MazeMap mazeMap)
	{
		setJMenuBar(createMenuBar());

		setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

		mazePanel = new MazePanel();
		setContentPane(createSplitPanel());
		setSize(FRAME_SIZE);
		setVisible(true);
		setResizable(false);

		mazePanel.setMaze(mazeMap);

	}

	private JMenuBar createMenuBar()
	{

		JMenu menu = new JMenu("Plik");

		JMenuItem menuItem = new JMenuItem("Wczytaj plik...", KeyEvent.VK_T);
		// menuItem.addActionListener(e -> {
		// JFileChooser fileChooser = new JFileChooser();
		// fileChooser.setFileFilter(new
		// FileNameExtensionFilter("RoboMaze map (*.roson)", "roson"));
		// if (fileChooser.showOpenDialog(CapoMazeVisualizer.this) ==
		// JFileChooser.APPROVE_OPTION) {
		// File file = fileChooser.getSelectedFile();
		// logger.debug("Opening file: " + file.getName());
		// Gson gson = new Gson();
		// try {
		// MazeMap mazeMap = gson.fromJson(new FileReader(file), MazeMap.class);
		// infoPanel.updateAgents(mazeMap);
		// } catch (FileNotFoundException e1) {
		// logger.debug("Could not read file: " + file.getName());
		// }
		// }
		// });

		JMenuItem menuItemRobots = new JMenuItem("Wyczysc roboty", KeyEvent.VK_T);
		menuItemRobots.addActionListener(new ActionListener()
		{
			public void actionPerformed(ActionEvent ev)
			{
				mazePanel.ramoveAllTrajectory();
			}
		});

		menu.add(menuItem);
		menu.add(menuItemRobots);

		JMenuBar menuBar = new JMenuBar();
		menuBar.add(menu);
		return menuBar;
	}

	private JSplitPane createSplitPanel()
	{
		JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, mazePanel, new Panel());
		splitPane.setDividerSize(5);
		splitPane.setDividerLocation(SPLIT_DIVIDER_LOCATION);
		splitPane.setEnabled(false);

		return splitPane;
	}

	@Override
	public void consumeMessage(Trajectory trajectory)
	{
		mazePanel.addTrajectory(trajectory);

	}

	@Override
	public void removeMassage(Trajectory trajectory)
	{
		mazePanel.removeTrajectory(trajectory);
	}

}
