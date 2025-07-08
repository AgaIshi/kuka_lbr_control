package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.Arrays;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import com.kuka.connectivity.fastRobotInterface.*;

public class CartesianImpedanceControl extends RoboticsAPIApplication {
	// members
	@Inject
	private LBR lbr_;
	private Controller lbr_controller_;
	
    @Inject
    @Named("Tool")
    private Tool mytool;

	// control mode
	private enum CONTROL_MODE {
		POSITION_CONTROL,
		JOINT_IMPEDANCE_CONTROL,
		CARTESIAN_IMPEDANCE_CONTROL;
	}

	// convert enum to string array, see https://stackoverflow.com/questions/13783295/getting-all-names-in-an-enum-as-a-string
	public static String[] getNames(Class<? extends Enum<?>> e) {
	    return Arrays.toString(e.getEnumConstants()).replaceAll("^.|.$", "").split(", ");
	}

	// FRI parameters 
	private String client_name_ = "172.31.1.148";
	private int send_period_ = 1;  // send period in ms

	private FRIConfiguration fri_configuration_;
	private FRISession fri_session_;
	private FRIJointOverlay fri_overlay_;

	private AbstractMotionControlMode control_mode_;
	private String[] control_modes_ = getNames(CONTROL_MODE.class);
	private ClientCommandMode command_mode_;
	private String[] command_modes_ = getNames(ClientCommandMode.class);
	// stiffness diagonal
	private double[] K = {1000.0,1000.0,1000.0,30.0,30.0,30.0};
	private double D0 = 1.0;
	
	private double ns_stiffness = 30.0;
	
	// methods
	public void request_user_config() {
		getLogger().info("Send period set to: " + send_period_);
		
		getLogger().info("Remote address set to: " + client_name_);
		
		control_mode_= 	new CartesianImpedanceControlMode();
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.X).setStiffness(K[0]);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.Y).setStiffness(K[1]);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.Z).setStiffness(K[2]);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.A).setStiffness(K[3]);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.B).setStiffness(K[4]);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.C).setStiffness(K[5]);
		
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.X).setDamping(D0);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.Y).setDamping(D0);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.Z).setDamping(D0);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.A).setDamping(D0);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.B).setDamping(D0);
		((CartesianImpedanceControlMode) control_mode_).parametrize(CartDOF.C).setDamping(D0);
		
		
		((CartesianImpedanceControlMode) control_mode_).setNullSpaceStiffness(ns_stiffness);
		((CartesianImpedanceControlMode) control_mode_).setNullSpaceDamping(1.0);
		
//		((CartesianImpedanceControlMode) control_mode_).setMaxControlForce(10.0,10.0,10.0,2.0,2.0,2.0,false);
		getLogger().info("Control mode set to: Cartesian Impedance Control");
		getLogger().info("Cartesian Stiffness set to: "+ K[0]+ " "+K[1]+" "+K[2]+" "+K[3]+" "+K[4]+" "+K[5]);
		getLogger().info("Nullspace Stiffness set to: "+ ns_stiffness);

		command_mode_ = ClientCommandMode.POSITION;
		getLogger().info("Client command mode set to: " + command_mode_.name());	
	}
	
	public void configure_fri() {
		fri_configuration_ = FRIConfiguration.createRemoteConfiguration(lbr_, client_name_);
		fri_configuration_.setSendPeriodMilliSec(send_period_);
		
        getLogger().info("Creating FRI connection to " + fri_configuration_.getHostName());
        getLogger().info(
			"SendPeriod: " + fri_configuration_.getSendPeriodMilliSec() + "ms |"
	        + " ReceiveMultiplier: " + fri_configuration_.getReceiveMultiplier()
        );
        
        fri_session_ = new FRISession(fri_configuration_);
        fri_overlay_ = new FRIJointOverlay(fri_session_, command_mode_);
        
        fri_session_.addFRISessionListener(new IFRISessionListener() {
	    	@Override
	    	public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
	    		getLogger().info("Session State change " + friChannelInformation.getFRISessionState().toString() );
	    	}
	
	    	@Override
	    	public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
		    	getLogger().info("Quality change signalled "+friChannelInformation.getQuality());
		    	getLogger().info("Jitter "+friChannelInformation.getJitter());
		    	getLogger().info("Latency "+friChannelInformation.getLatency());
	    	}
    	});        
        
        // try to connect
        try {
        	fri_session_.await(60, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
        	getLogger().error(e.getLocalizedMessage());
        	getLogger().error("Connection timeout: Current Timeout limit = 60 sec");
        	return;
        }
        
        getLogger().info("FRI connection established.");
	}
	
	@Override
	public void initialize() {
		ObjectFrame lbr_flange = lbr_.getFlange();
		mytool.attachTo(lbr_flange);
		getLogger().info("End Effector position:" + lbr_flange.getX()/1000.0 + " " + lbr_flange.getY()/1000.0 + " " + lbr_flange.getZ()/1000.0);
		
        lbr_controller_ = (Controller) getContext().getControllers().toArray()[0];
        lbr_ = (LBR) lbr_controller_.getDevices().toArray()[0];
		
        // set FRI parameters
		request_user_config();
		
		// configure the FRI
		configure_fri();
	}

	@Override
	public void run() {
		// run the FRI
		lbr_.move(positionHold(control_mode_, -1, null).addMotionOverlay(fri_overlay_));			
		return;
	}
	
	@Override
	public void dispose() {
		// close connection
		getLogger().info("Disposing FRI session.");
		fri_session_.close();

		super.dispose();
	}
	
	/**
	 * main
	 * 
	 * @param args
	 */
	public static void main(final String[] args) {
		CartesianImpedanceControl app = new CartesianImpedanceControl();
		app.runApplication();
	}
}
