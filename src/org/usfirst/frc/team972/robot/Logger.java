package org.usfirst.frc.team972.robot;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Calendar;

public class Logger {

	static String directory = "";

	public static void init() {
		setNewDirectory();
	}
	
	public static void setNewDirectory(String directoryName) {
		new File(Constants.LOGGER_LOCATION).mkdir();
		new File(Constants.LOGGER_LOCATION + "/" + directoryName).mkdir();
		directory = directoryName;
	}

	public static void setNewDirectory() {
		setNewDirectory(new SimpleDateFormat("MM-dd-yy_HH:mm:ss").format(Calendar.getInstance().getTime()));
	}

	public static void log(String fileName, String message, boolean error) {
		try {
			PrintWriter out = new PrintWriter(new BufferedWriter(
					new FileWriter(Constants.LOGGER_LOCATION + "/" + directory + "/" + fileName + ".txt", true)));
			out.println(message);
			out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

		if (error) {
			logError(fileName + ": " + message);
		}
	}

	public static void log(String fileName, String message) {
		try {
			PrintWriter out = new PrintWriter(new BufferedWriter(
					new FileWriter(Constants.LOGGER_LOCATION + "/" + directory + "/" + fileName + ".txt", true)));
			out.println(message);
			out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public static void logError(String message) {
		try {
			PrintWriter errorOut = new PrintWriter(
					new BufferedWriter(new FileWriter(Constants.LOGGER_LOCATION + "/ErrorLog.txt", true)));
			errorOut.println(message);
			errorOut.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static double[] readLog(String fileName, double time) {
		try {
			String[] tokens;
			String line;
			double[] values = { 0, 0 };
			BufferedReader fileReader = 
	                new BufferedReader(new FileReader(Constants.LOGGER_LOCATION + "/" + directory + "/" + fileName + ".txt"));
			while((line = fileReader.readLine()) != null) {
				tokens = line.split("x=");
				double line_time = Double.parseDouble(tokens[0].substring(2));
				if (Math.abs(line_time - time) < 0.011) {
					tokens = tokens[1].split("y=");
					String x_str = tokens[0];
					String y_str = tokens[1];
					tokens = tokens[1].split("theta=");
					String theta_str = tokens[1];
					values[0] = Double.parseDouble(x_str);
					values[1] = Double.parseDouble(y_str);
					values[2] = Double.parseDouble(theta_str);
					fileReader.close();
					return values;
				}
			}
			fileReader.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}
	
	public static void deleteFile(String fileName) {
		File file = new File(Constants.LOGGER_LOCATION + "/" + directory + "/" + fileName + ".txt");
    	try {
    		file.delete();
    		logError("Deleted file with name: " + fileName);
		} catch (Exception e) {
    		e.printStackTrace();
    	}
	}
	
	public static void renameFile(String fileName, String newFileName) {
		File oldFile = new File(Constants.LOGGER_LOCATION + "/" + directory + "/" + fileName + ".txt");
		File newFile = new File(Constants.LOGGER_LOCATION + "/" + directory + "/" + newFileName + ".txt");
    	try {
    		if (!oldFile.renameTo(newFile)) { //makes sure that even if it fails we at least delete the file which is necessary when logging the motionprofiling stuff
    			deleteFile(fileName);
    		}
		} catch (Exception e) {
    		e.printStackTrace();
    	}
	}
}
