package android.util;

public final class Log {
    public static final int ASSERT = 7;
    public static final int DEBUG = 3;
    public static final int ERROR = 6;
    public static final int INFO = 4;
    public static final int VERBOSE = 2;
    public static final int WARN = 5;

    

    public static int println(int priority, String tag, String msg) {
	
		String priorityString = 
			priority == ASSERT? "ASSERT" :
			priority == DEBUG? "DEBUG" :
			priority == ERROR? "ERROR" :
			priority == INFO? "INFO" :
			priority == VERBOSE? "VERBOSE" :
			priority == WARN? "WARN" :
			"UNKNOWN";
			
		String s = "" + priorityString + "  " + tag + "  " + msg;
		System.out.println(s);

		return 0;
	
    }
}
