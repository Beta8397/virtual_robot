package com.qualcomm.hardware.gobilda;

import com.qualcomm.hardware.CommonOdometry;

public class GoBildaPinpointDriverInternal extends GoBildaPinpointDriver{

    public GoBildaPinpointDriverInternal(CommonOdometry odo){
        super(odo);
    }

    @Override
    public synchronized void internalUpdate(boolean updateEncoders, boolean headingOnly){
        super.internalUpdate(updateEncoders, headingOnly);
    }

    public synchronized void resetEncoders(){
        xEncoderValue = 0;
        yEncoderValue = 0;
    }


}
