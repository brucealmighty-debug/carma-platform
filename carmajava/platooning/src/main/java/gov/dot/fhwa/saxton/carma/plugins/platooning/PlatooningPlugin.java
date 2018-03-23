/*
 * Copyright (C) 2018 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import cav_msgs.MobilityOperation;
import cav_msgs.MobilityRequest;
import cav_msgs.MobilityResponse;
import cav_msgs.PlatooningInfo;
import cav_msgs.SpeedAccel;
import gov.dot.fhwa.saxton.carma.guidance.arbitrator.TrajectoryPlanningResponse;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.AbstractPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IStrategicPlugin;
import gov.dot.fhwa.saxton.carma.guidance.plugins.PluginServiceLocator;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

public class PlatooningPlugin extends AbstractPlugin implements IStrategicPlugin {

    // TODO the plugin should use interface manager once rosjava multi-thread service call is fixed
    protected final String SPEED_CMD_CAPABILITY = "/saxton_cav/drivers/srx_controller/control/cmd_speed";
    protected final String PLATOONING_FLAG      = "PLATOONING";

    // initialize pubs/subs
    protected IPublisher<MobilityRequest>   mobilityRequestPublisher;
    protected IPublisher<MobilityOperation> mobilityOperationPublisher;
    protected IPublisher<PlatooningInfo>    platooningInfoPublisher;
    protected ISubscriber<SpeedAccel>       cmdSpeedSub;
    
    
    // following parameters are for CACC platooning
    protected double maxAccel              = 2.5;
    protected double minimumManeuverLength = 15.0;
    protected double timeHeadway           = 1.8;
    protected double standStillGap         = 7.0;
    protected double kpPID                 = 1.5;
    protected double kiPID                 = 0.0;
    protected double kdPID                 = 0.1;
    protected double messageTimeoutFactor  = 750;
    protected int    messageIntervalLength = 100;
    
    // following parameters are for leader selection
    protected double lowerBoundary = 1.65;
    protected double upperBoundary = 1.75;
    protected double maxSpacing    = 2.0;
    protected double minSpacing    = 1.9;
    protected double minGap        = 12.0;
    protected double maxGap        = 14.0;
    protected int    algorithmType = 1;
    
    // platooning plug-in components
    protected IPlatooningState state                  = null;
    protected Thread           stateThread            = null;
    protected CommandGenerator commandGenerator       = null;
    protected Thread           commandGeneratorThread = null;
    protected PlatoonManager   platoonManager         = null;
    protected Thread           platoonManagerThread   = null;
    
    public PlatooningPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
        version.setName("CACC Platooning Plugin");
        version.setMajorRevision(1);
        version.setIntermediateRevision(0);
        version.setMinorRevision(0);
    }

    @Override
    public void onInitialize() {
        log.info("CACC platooning plugin is initializing...");
        
        // initialize parameters of platooning plug-in
        maxAccel              = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_accel", 2.5);
        minimumManeuverLength = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_maneuver_length", 15.0);
        timeHeadway           = pluginServiceLocator.getParameterSource().getDouble("~platooning_desired_time_headway", 1.8);
        standStillGap         = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_gap", 7.0);
        kpPID                 = pluginServiceLocator.getParameterSource().getDouble("~platooning_Kp", 1.5);
        kiPID                 = pluginServiceLocator.getParameterSource().getDouble("~platooning_Ki", 0.0);
        kdPID                 = pluginServiceLocator.getParameterSource().getDouble("~platooning_Kd", 0.1);
        messageTimeoutFactor  = pluginServiceLocator.getParameterSource().getDouble("~platooning_status_timeout_factor", 2.5);
        lowerBoundary         = pluginServiceLocator.getParameterSource().getDouble("~platooning_lower_boundary", 1.65);
        upperBoundary         = pluginServiceLocator.getParameterSource().getDouble("~platooning_upper_boundary", 1.75);
        maxSpacing            = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_spacing", 2.0);
        minSpacing            = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_spacing", 1.9);
        minGap                = pluginServiceLocator.getParameterSource().getDouble("~platooning_min_gap", 12.0);
        maxGap                = pluginServiceLocator.getParameterSource().getDouble("~platooning_max_gap", 14.0);
        messageIntervalLength = pluginServiceLocator.getParameterSource().getInteger("~platooning_status_interval", 100);
        algorithmType         = pluginServiceLocator.getParameterSource().getInteger("~algorithm_type", 1);
        //log all loaded parameters
        log.debug("Load param maxAccel = " + maxAccel);
        log.debug("Load param minimumManeuverLength = " + minimumManeuverLength);
        log.debug("Load param timeHeadway = " + timeHeadway);
        log.debug("Load param standStillGap = " + standStillGap);
        log.debug("Load param for speed PID controller: [p = " + kpPID + ", i = " + kiPID + ", d = " + kdPID + "]");
        log.debug("Load param messageIntervalLength = " + messageIntervalLength);
        log.debug("Load param messageTimeoutFactor = " + messageTimeoutFactor);        
        log.debug("Load param lowerBoundary = " + lowerBoundary);        
        log.debug("Load param upperBoundary = " + upperBoundary);        
        log.debug("Load param maxSpacing = " + maxSpacing);        
        log.debug("Load param minSpacing = " + minSpacing);        
        log.debug("Load param minGap = " + minGap);        
        log.debug("Load param maxGap = " + maxGap);        
        log.debug("Load param algorithmType = " + algorithmType);
        
        // initialize necessary pubs/subs
        mobilityRequestPublisher   = pubSubService.getPublisherForTopic("outgoing_mobility_request", MobilityRequest._TYPE);
        mobilityOperationPublisher = pubSubService.getPublisherForTopic("outgoing_mobility_operation", MobilityOperation._TYPE);
        platooningInfoPublisher    = pubSubService.getPublisherForTopic("platooning_info", PlatooningInfo._TYPE);
        cmdSpeedSub                = pubSubService.getSubscriberForTopic(SPEED_CMD_CAPABILITY, SpeedAccel._TYPE);
        
        //TODO register with MobilityRouter
        log.info("CACC platooning plugin is initialized.");
    }

    @Override
    public void onResume() {
        log.info("CACC platooning plugin resume to operate.");
        // reset plug-in's sub-components
        this.setState(new StandbyState(this, log, pluginServiceLocator));
        if(platoonManagerThread == null) {
            platoonManager       = new PlatoonManager(this, log, pluginServiceLocator);
            platoonManagerThread = new Thread(platoonManager);
            platoonManagerThread.setName("Platooning List Manager");
            platoonManagerThread.start();
            log.debug("Started platoonManagerThread");
        }
        if(commandGeneratorThread == null) {
            commandGenerator       = new CommandGenerator(this, log, pluginServiceLocator);
            commandGeneratorThread = new Thread(commandGenerator);
            commandGeneratorThread.setName("Platooning Command Generator");
            commandGeneratorThread.start();
            log.debug("Started commandGeneratorThread");
        }
        log.info("The current CACC plugin state is " + this.state.toString());
        this.setAvailability(true);
    }

    @Override
    public void onSuspend() {
        this.setAvailability(false);
        if(stateThread != null) {
            stateThread.interrupt();
            stateThread = null;
        }
        if(commandGeneratorThread != null) {
            commandGeneratorThread.interrupt();
            commandGeneratorThread = null;
        }
        if(platoonManagerThread != null) {
            platoonManagerThread.interrupt();
            platoonManagerThread = null;
        }
        log.info("CACC platooning plugin is suspended.");
    }
    
    @Override
    public void onTerminate() {
        // NO-OP
    }
    
    @Override
    public void loop() throws InterruptedException {
        // publish platooning information message for the usage of UI
        publishPlatooningInfo();
    }

    @Override
    public TrajectoryPlanningResponse planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        log.info("Plan Trajectory from " + traj.toString() + " in state " + state.toString());
        return this.state.planTrajectory(traj, expectedEntrySpeed);
    }
    
    // TODO The following method belongs to the mobility message handler interface that guidance package will provide
    public void handleMobilityOperationMessage(MobilityOperation msg) {
        this.state.onMobilityOperationMessage(msg);
    }
    
    // TODO The following method belongs to the mobility message handler interface that guidance package will provide
    public boolean handleMobilityRequestMessage(MobilityRequest msg) {
        return this.state.onMobilityRequestMessgae(msg);
    }
    
    // TODO The following method belongs to the mobility message handler interface that guidance package will provide
    public void headleMobilityResponse(MobilityResponse msg) {
        this.state.onMobilityResponseMessage(msg);
    }
    
    // Set state for the current plug-in
    protected void setState(IPlatooningState state) {
        if(stateThread != null) {
            stateThread.interrupt();
            stateThread = null;
        }
        String previousState = this.state == null ? "NULL" : this.state.toString();
        log.info("Platooning plugin is changing from " + previousState + " state to " + state.toString() + " state");
        this.state = state;
        stateThread = new Thread(state);
        stateThread.start();
        log.debug("Started stateThread");
    }
    
    // TODO Once this plug-in is finished, we should replace them with actual data
    private void publishPlatooningInfo() {
        if (platooningInfoPublisher != null) {
            PlatooningInfo info = platooningInfoPublisher.newMessage();
            info.setState(PlatooningInfo.FOLLOWER);
            info.setPlatoonId("b937d2f6-e618-4867-920b-c1f74f98ef1f");
            info.setSize((byte) 5);
            info.setSizeLimit((byte) 10);
            info.setLeaderId("DOT-40053");
            info.setLeaderDowntrackDistance((float) 50.3);
            info.setLeaderCmdSpeed((float) 5.6);
            info.setHostPlatoonPosition((byte) 4);
            info.setHostCmdSpeed((float) 5.4);
            info.setDesiredGap((float) 45.8);
            platooningInfoPublisher.publish(info);
        }
    }
    
    // The following getters will be helpful on doing unit tests, because it will let the plugin's
    // other components not depend on the actual platooning plug-in class. 
    protected CommandGenerator getCommandGenerator() {
        return commandGenerator;
    }

    protected PlatoonManager getPlatoonManager() {
        return platoonManager;
    }
    
    protected double getTimeHeadway() {
        return timeHeadway;
    }

    protected double getStandStillGap() {
        return standStillGap;
    }

    protected double getKpPID() {
        return kpPID;
    }

    protected double getKiPID() {
        return kiPID;
    }

    protected double getKdPID() {
        return kdPID;
    }
    
    protected double getMaxAccel() {
        return maxAccel;
    }
    
    protected IManeuverInputs getManeuverInputs() {
        return pluginServiceLocator.getManeuverPlanner().getManeuverInputs();
    }
    
    protected double getLowerBoundary() {
        return lowerBoundary;
    }

    protected double getUpperBoundary() {
        return upperBoundary;
    }

    protected double getMaxSpacing() {
        return maxSpacing;
    }

    protected double getMinSpacing() {
        return minSpacing;
    }

    protected double getMinGap() {
        return minGap;
    }

    protected double getMaxGap() {
        return maxGap;
    }
    
    protected int getAlgorithmType() {
        return algorithmType;
    }
    
}
