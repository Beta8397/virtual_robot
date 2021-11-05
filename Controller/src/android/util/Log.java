/*
 * Copyright (C) 2006 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Modified by Team Beta for use in the Virtual Robot simulator.
 */

package android.util;

public final class Log {
    public static final int ASSERT = 7;
    public static final int DEBUG = 3;
    public static final int ERROR = 6;
    public static final int INFO = 4;
    public static final int VERBOSE = 2;
    public static final int WARN = 5;

    
	public static int d(String tag, String msg) {
		return println(DEBUG, tag, msg);
	}

	public static int e(String tag, String msg) {
		return println(ERROR, tag, msg);
	}
    
    public static int i(String tag, String msg) {
		return println(INFO, tag, msg);
	}

	public static int v(String tag, String msg) {
		return println(VERBOSE, tag, msg);
	}

	public static int w(String tag, String msg) {
		return println(WARN, tag, msg);
	}

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
