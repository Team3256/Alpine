package frc.team3256.robot.constants;

import java.util.logging.Level;

public class LoggingConstants {


    // ******* DEBUG SETTINGS ******* //

    //Make sure these are both WARNING in Competition
    //Ideally CONSOLE should be at most warning, too cluttered otherwise
    public static Level LOG_LEVEL = Level.WARNING;
    public static Level CONSOLE_LEVEL = Level.WARNING;

    //Uses Internal Storage for normal logging, be careful of
    //using too much storage - Normally should be false.
    public static boolean FORCE_NORMAL_INTERNAL = false;

    // ******************************* //


    //Max Number of Files
    public static final int TXT_LOG_MAX_FILES = 10;
    public static final int HTML_LOG_MAX_FILES = 3;

    //Normal Max File Sizes
    public static final int TXT_LOG_MAX_SIZE = 100000000;     // In Bytes
    public static final int HTML_LOG_MAX_SIZE = 100000000; // In Bytes

    //File Names (%g is for Numbering of Files)
    public static final String TXT_FILE_NAME = "TextLog%g.txt";
    public static final String HTML_FILE_NAME = "HtmlLog%g.html";


    //Emergency File Settings (NO USB)
    public static final int EMERGENCY_TXT_MAX_FILES = 3;
    public static final int EMERGENCY_TXT_MAX_SIZE = 3000000; //In Bytes


}
