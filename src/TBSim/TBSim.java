/*
 * TBSim.java
 */

package TBSim;

import java.io.*;
import java.awt.*;
import java.net.*;
import java.awt.event.*;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.util.*;

/**
 * Application that runs a control system in
 * simulation.
 * <P>
 * To run this program, first ensure you have your CLASSPATH set correctly,
 * then type "java TBSim.TBSim".
 * <P>
 * For more detailed information, see the
 * <A HREF="docs/index.html">TBSim page</A>.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A> 
 * (c)1997 Georgia Tech Research Corporation
 *
 * @author Tucker Balch
 * @version $Revision: 1.3 $
 */

public class TBSim extends Frame 
	implements ActionListener, ItemListener
	{

	  private static final String fileMenuName = "File";
	  // only used if there is no default menu
	private static final String helpMenuName = "Help"; 
	private static final String viewMenuName = "View";
	private static final String loadCommandName = "Load";
	private static final String printCommandName = "Print";
	private static final String quitCommandName = "Quit";
	private static final String graphicsCommandName = "Graphics";
	private static final String robotIDsCommandName = "Robot IDs";
	private static final String trailsCommandName = "robot trails";
	private static final String stateCommandName = "robot state/potentials";
	private static final String infoCommandName = "object info";
	private static final String iconsCommandName = "icons";
	private static final String resetCommandName = "reset/reload";
	private static final String startCommandName = "start/resume";
	private static final String pauseCommandName = "pause";
	private static final String statsCommandName = "Runtime Stats";
	private static final String aboutCommandName = "About";
	private static final String jokeCommandName = "     ";
	
	private static final String descriptionFileSuffix = ".dsc";

	  private Frame simFrame;
	private SimulationCanvas simulation;
	private String	dsc_file;
	private	int	height;
	private	int	width;
	private	String	current_directory;
	//private	CheckboxMenuItem view_graphics;
	private	CheckboxMenuItem robot_ids;
	private	CheckboxMenuItem robot_trails;
	private	CheckboxMenuItem robot_state;
	private	CheckboxMenuItem object_state;
	private	CheckboxMenuItem icons;

	  public TBSim() {
	    this(null, 200, 100, false);
	  }
	  
	  public TBSim(String file) {
	    this(file, 500, 500, false);
	  }
	  
	  public TBSim(String file, int width, int height) {
	    this(file, width, height, true);
	  }



	/**
	 * Set up the frame and buttons.
	 */
	public TBSim(String file, int width, int height, 
		     boolean preserveSize)
		{
		simFrame = new Frame("TBSim");

		/*--- Set the title ---*/
		// try to find our hostnam first
		InetAddress this_host;
		try
			{
			this_host = InetAddress.getLocalHost();
			}
		catch (Exception e)
			{
			this_host = null;
			}
		String host_name = "unknown host";
		if (this_host != null)
			host_name = this_host.getHostName();
		simFrame.setTitle("TBSim"+" ("+host_name+")");

		/*--- Set up the menu bar ---*/
		MenuBar mb = new MenuBar();

		Menu mf = new Menu(fileMenuName);
		mf.add(loadCommandName);
		mf.add(printCommandName);
		mf.add(quitCommandName);
		mf.addActionListener(this);
		mb.add(mf);

		Menu mv = new Menu(viewMenuName);
		//view_graphics = new CheckboxMenuItem(graphicsCommandName);
		//mv.add(view_graphics);
		//view_graphics.addItemListener(this);
		robot_ids = new CheckboxMenuItem(robotIDsCommandName);
		mv.add(robot_ids);
		robot_ids.addItemListener(this);
		robot_trails = new CheckboxMenuItem(trailsCommandName);
		mv.add(robot_trails);
		robot_trails.addItemListener(this);
		robot_state = new CheckboxMenuItem(stateCommandName);
		mv.add(robot_state);
		robot_state.addItemListener(this);
		object_state = new CheckboxMenuItem(infoCommandName);
		mv.add(object_state);
		object_state.addItemListener(this);
		icons = new CheckboxMenuItem(iconsCommandName);
		mv.add(icons);
		icons.addItemListener(this);

		mb.add(mv);

		Menu hm = mb.getHelpMenu();
		if (hm == null) {
		  hm = new Menu(helpMenuName);
		  mb.setHelpMenu(hm);
		}
		hm.add(statsCommandName);
		hm.add(aboutCommandName);
		hm.add(jokeCommandName);
		hm.addActionListener(this);

		simFrame.setMenuBar(mb);

		/*--- Set up the buttons ---*/
		Panel button_area = new Panel();
		Button start_button = new Button(resetCommandName);
		start_button.addActionListener(this);
		button_area.add(start_button);
		Button resume_button = new Button(startCommandName);
		resume_button.addActionListener(this);
		button_area.add(resume_button);
		Button pause_button = new Button(pauseCommandName);
		pause_button.addActionListener(this);
		button_area.add(pause_button);
		simFrame.add("North",button_area);

		/*--- Set up the Graphical Area ---*/
		Panel playing_field_panel = new Panel();

		dsc_file = file;

		simulation = 
			new SimulationCanvas(simFrame,width,height,dsc_file,preserveSize);
		playing_field_panel.add(simulation);
		simFrame.add("South",playing_field_panel);

		/*--- Pack Everything ---*/
		simFrame.pack();
		simFrame.setResizable(false);

		/*--- tell the simulation to load and run ---*/
		if (dsc_file!=null) simulation.reset();
	
		/*--- set the menu options we learned from a dsc file ---*/
		robot_ids.setState(simulation.draw_ids);
		robot_trails.setState(simulation.draw_trails);
		robot_state.setState(simulation.draw_robot_state);
		object_state.setState(simulation.draw_object_state);
		icons.setState(simulation.draw_icons);

		if (simulation.descriptionLoaded())// only if loaded ok.
			{
			simulation.start();
			}
		}

	/**
	 * Handle checkbox events
	 */
	public void itemStateChanged(ItemEvent e)
		{
		String item = e.getItem().toString();

		//if (item == graphicsCommandName)
		//	{
		//	simulation.setGraphics(view_graphics.getState());
		//	}
		if (item == robotIDsCommandName)
			simulation.setDrawIDs(robot_ids.getState());
		if (item == trailsCommandName)
			simulation.setDrawTrails(robot_trails.getState());
		if (item == stateCommandName)
			{
			simulation.setDrawRobotState(robot_state.getState());
			}
		if (item == infoCommandName)
			simulation.setDrawObjectState(object_state.getState());
		if (item == iconsCommandName)
			simulation.setDrawIcons(icons.getState());
		}

	/**
	 * Handle button pushes.
	 */
	public void actionPerformed(ActionEvent e)
		{
		String command = e.getActionCommand();

		/*--- Load ---*/
		if (command==loadCommandName)
			{
			FileDialog fd = new FileDialog(simFrame,
				"Load New Description File",
				FileDialog.LOAD);

			/*--- try to filter based on extension ---*/
			// curently, this has no effect under linux and Irix
			FilenameFilter filt = 
				new FilenameFilterByEnding(descriptionFileSuffix);
			fd.setFilenameFilter(filt);
			if (current_directory != null)
				fd.setDirectory(current_directory);
			fd.show();
			String tmpname = fd.getFile();
			if (tmpname==null) tmpname = "";
			current_directory = fd.getDirectory();
			if (current_directory!=null) 
				tmpname = current_directory.concat(tmpname);
			if (tmpname!=null)
				{
				dsc_file = tmpname;
				simulation.load(dsc_file);
				//now update the checkboxes on the menu
				/*--- set the menu options we learned from a dsc file ---*/
				robot_ids.setState(simulation.draw_ids);
				robot_trails.setState(simulation.draw_trails);
				robot_state.setState(simulation.draw_robot_state);
				object_state.setState(simulation.draw_object_state);
				icons.setState(simulation.draw_icons);
				}
			fd.dispose();
			}

		/*--- Quit ---*/
		if (command==quitCommandName)
			{
			simulation.quit();
			System.exit(0);
			}

		/*--- Runtime Stats ---*/
		if (command==statsCommandName)
			{
			simulation.showRuntimeStats();
			}

		/*--- About ---*/
		else if(command == aboutCommandName)
			{
			Dialog tmp = new DialogMessage(simFrame,
				"About TBSim, the TeamBots Simulator",
				TBVersion.longReport() + "\n");
			}

		/*--- Avoid Cursor ---*/
		else if(command == jokeCommandName)
			{
			Dialog tmp = new DialogMessageJoke(simFrame,
				"Avoid Cursor",
				"An implementation of the avoid_cursor\nmotor schema.");
			}

		/*--- Robot IDs ---*/
		else if(command == robotIDsCommandName)
			{
			System.out.println(robot_ids.getState());
			}

		/*--- Print ---*/
		else if (command == printCommandName)
			{
			PrintJob pjob = simFrame.getToolkit().getPrintJob(simFrame,
				"Print?", null);
			if (pjob != null)
				{
				Graphics pg = pjob.getGraphics();
				if (pg != null)
					simFrame.printAll(pg);
					pg.dispose();
				}
			pjob.end();
			}

		/*--- Start/Resume ---*/
		else if (command == startCommandName)
			{
			simulation.start();
			}

		/*--- Reset ---*/
		else if (command == resetCommandName)
			{
			simulation.reset();
			}

		/*--- Pause ---*/
		else if (command == pauseCommandName)
			{
			simulation.pause();
			}
		}

	  public void show() {
	    simFrame.setVisible(true);
	  }
	  
        /**
	 * Main for TBSim.
         */
	public static void main(String[] args) 
		{
		  long runtime = 0;
		  boolean gotSize = false;
		  int width = -1;
		  int height = -1;
		  String dsc_file = null;

		/*--- check the arguments ---*/
		if (args.length >= 1)
			{
			if (args[0].equalsIgnoreCase("-version"))
				{
				System.out.println(
					TBVersion.longReport());
				System.exit(0);
				}
			else {
				dsc_file=args[0];
			
			}
			}
		System.out.println(TBVersion.shortReport());
		try
			{
			  if (args.length >= 3)
			    {
			      width = Integer.parseInt(args[1]);
			      height = Integer.parseInt(args[2]);
			      gotSize = true;
			    }
			}
		catch (Exception e)
			{
			System.out.println(
				"usage: java TBSim.TBSim [-version] [descriptionfile] [width height]");
			}
		/*--- make the window ---*/
		
		TBSim jbs = null;
		if (gotSize)
			jbs = new TBSim(dsc_file, width, height);
		else if (dsc_file != null)
			jbs = new TBSim(dsc_file);
		else
			jbs = new TBSim();
		jbs.show();
		}
	}
