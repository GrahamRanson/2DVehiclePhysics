-- This is an adaption of SpaceJack's vehicle physics code found here - https://github.com/spacejack/carphysics2d

--- Required libraries.

--- Required libraries.
local Vector2 = require( "2DVehiclePhysics.vector2" )
local Maths = require( "2DVehiclePhysics.maths" ).new()

-- Localised functions.
local deg = math.deg
local abs = math.abs
local max = math.max
local min = math.min
local sin = math.sin
local cos = math.cos
local atan2 = math.atan2

-- Localised values.

--- Class creation.
local Controller = {}

--- Initiates a new Controller object.
-- @param options The options for the Controller.
-- @return The new Controller.
function Controller.new( options )

    local options = options or {}

	-- Create ourselves
	local self = {}

    --  Controller state variables
	self.heading = options.heading or 0  -- angle car is pointed at (radians)
	self.position = Vector2.new( options.x, options.y )  -- metres in world coords
	self.velocity = Vector2.new()  -- m/s in world coords
	self.velocity_c = Vector2.new()  -- m/s in local car coords (x is forward y is sideways)
	self.accel = Vector2.new()  -- acceleration in world coords
	self.accel_c = Vector2.new()  -- accleration in local car coords
	self.absVel = 0 -- absolute velocity m/s
	self.yawRate = 0 -- angular velocity in radians
	self.steer = 0 -- amount of steering input (-1.0..1.0)
	self.steerAngle = 0 -- actual front wheel steer angle (-maxSteer..maxSteer)

    --  State of inputs
	self.inputs =
    {
        brake = 0,
        ebrake = 0,
        throttle = 0,
        left = 0,
    	right = 0
    }

    --  Use input smoothing (on by default)
	self.smoothSteer = options.smoothSteer
    if self.smoothSteer == nil then
        self.smoothSteer = true
    end

	--  Use safe steering (angle limited by speed)
	self.safeSteer = options.safeSteer
    if self.safeSteer == nil then
        self.safeSteer = true
    end

    --  Other static values to be computed from config
	self.inertia = 0 -- will be = mass
	self.wheelBase = 0 -- set from axle to CG lengths
	self.axleWeightRatioFront = 0 -- % car weight on the front axle
	self.axleWeightRatioRear = 0 -- % car weight on the rear axle

    self.config =
    {
        gravity = options.gravity or 9.81, -- m/s^2
    	mass = options.mass or 1200.0, -- kg
    	inertiaScale = options.inertiaScale or 1.0, -- Multiply by mass for inertia
    	halfWidth = options.halfWidth or 0.8, -- Centre to side of chassis (metres)
        cgToFront = options.cgToFront or 2.0, -- Centre of gravity to front of chassis (metres)
    	cgToRear = options.cgToRear or 2.0, -- Centre of gravity to rear of chassis
    	cgToFrontAxle = options.cgToFrontAxle or 1.25, -- Centre gravity to front axle
    	cgToRearAxle = options.cgToRearAxle or 1.25, -- Centre gravity to rear axle
    	cgHeight = options.cgHeight or 0.55, -- Centre gravity height
        wheelRadius = options.wheelRadius or 0.3, -- Includes tire (also represents height of axle)
    	wheelWidth = options.wheelWidth or 0.2, -- Used for render only
    	tireGrip = options.tireGrip or 2, -- How much grip tires have
    	lockGrip = ( type( options.lockGrip ) == "number" ) and Maths.clamp( options.lockGrip, 0.01, 1.0 ) or 0.7, -- % of grip available when wheel is locked
    	engineForce = options.engineForce or 8000,
    	brakeForce = options.brakeForce or 12000,
        brakeActsAsReverse = options.brakeActsAsReverse or false, -- If true then holding brake down for long enough will flip car into reverse
        weightTransfer = ( type( options.weightTransfer ) == "number" ) and options.weightTransfer or 0.2, -- How much weight is transferred during acceleration/braking
    	maxSteer = options.maxSteer or 0.6, -- Maximum steering angle in radians
    	cornerStiffnessFront = options.cornerStiffnessFront or 5.0,
    	cornerStiffnessRear = options.cornerStiffnessRear or 5.2,
    	airResist = ( type( options.airResist == "number" ) ) and options.airResist or 2.5, -- air resistance (* vel)
    	rollResist = ( type( options.rollResist == "number" ) ) and options.rollResist or 8.0, -- rolling resistance force (* vel),
        yawFactor = ( type( options.yawFactor == "number" ) ) and options.yawFactor or 0.1 -- Factor to reduce the yaw amount by that I added as it didn't seem to work well without
    }
    self.config.eBrakeForce = options.eBrakeForce or self.config.brakeForce / 2.5

    self.inertia = self.config.mass * self.config.inertiaScale
	self.wheelBase = self.config.cgToFrontAxle + self.config.cgToRearAxle
	self.axleWeightRatioFront = self.config.cgToRearAxle / self.wheelBase -- % car weight on the front axle
	self.axleWeightRatioRear = self.config.cgToFrontAxle / self.wheelBase -- % car weight on the rear axle

    function self.applySmoothSteer( steerInput, dt )
    	local steer = 0

    	if abs( steerInput ) > 0.001 then
    		--  Move toward steering input
    		steer = Maths.clamp( self.steer + steerInput * dt * 2.0, -1.0, 1.0 ) -- -inp.right, inp.left);
    	else

    		--  No steer input - move toward centre (0)
    		if self.steer > 0 then
    			steer = max(self.steer - dt * 1.0, 0);
    		elseif self.steer < 0 then
    			steer = min(self.steer + dt * 1.0, 0);
    		end
    	end

    	return steer
    end

    function self.applySafeSteer( steerInput )
    	local avel = min( self.absVel, 250.0 ) -- m/s
    	local steer = steerInput * ( 1.0 - ( avel / 280.0 ) )
    	return steer
    end

    function self.update( dtms )

        local dt = dtms / 10.0 -- delta T in seconds

    	self.throttle = self.inputs.throttle
    	self.brake = self.inputs.brake

    	local steerInput = ( self.inputs.left or 0 ) - ( self.inputs.right or 0 )

    	--  Perform filtering on steering...
    	if self.smoothSteer then
    		self.steer = self.applySmoothSteer( steerInput, dt );
    	else
    		self.steer = steerInput
        end

    	if self.safeSteer then
    		self.steer = self.applySafeSteer( self.steer );
        end

    	--  Now set the actual steering angle
    	self.steerAngle = self.steer * self.config.maxSteer;

        self.doPhysics( dt )

    end

    function self.doPhysics( dt )

        -- Shorthand
        local cfg = self.config;


        -- Pre-calc heading vector
        local sn = sin( self.heading );
        local cs = cos( self.heading );


        -- Get velocity in local car coordinates
        self.velocity_c.x = cs * self.velocity.x + sn * self.velocity.y;
        self.velocity_c.y = cs * self.velocity.y - sn * self.velocity.x;

        -- Weight on axles based on centre of gravity and weight shift due to forward/reverse acceleration
        local axleWeightFront = cfg.mass * (self.axleWeightRatioFront * cfg.gravity - cfg.weightTransfer * self.accel_c.x * cfg.cgHeight / self.wheelBase);
        local axleWeightRear = cfg.mass * (self.axleWeightRatioRear * cfg.gravity + cfg.weightTransfer * self.accel_c.x * cfg.cgHeight / self.wheelBase);

        -- Resulting velocity of the wheels as result of the yaw rate of the car body.
        -- v = yawrate * r where r is distance from axle to CG and yawRate (angular velocity) in rad/s.
        local yawSpeedFront = cfg.cgToFrontAxle * self.yawRate;
        local yawSpeedRear = -cfg.cgToRearAxle * self.yawRate;

        -- Calculate slip angles for front and rear wheels (a.k.a. alpha)
        local slipAngleFront = atan2( self.velocity_c.y + yawSpeedFront, abs( self.velocity_c.x ) ) - Maths.sign( self.velocity_c.x ) * self.steerAngle;

        local slipAngleRear  = atan2( self.velocity_c.y + yawSpeedRear,  abs( self.velocity_c.x ) );

        local tireGripFront = cfg.tireGrip;
        local tireGripRear = cfg.tireGrip * (1.0 - self.inputs.ebrake * (1.0 - cfg.lockGrip)); -- reduce rear grip when ebrake is on

        local frictionForceFront_cy = Maths.clamp(-cfg.cornerStiffnessFront * slipAngleFront, -tireGripFront, tireGripFront) * axleWeightFront;
        local frictionForceRear_cy = Maths.clamp(-cfg.cornerStiffnessRear * slipAngleRear, -tireGripRear, tireGripRear) * axleWeightRear;

        --  Get amount of brake/throttle from our inputs
        local brake = min( self.inputs.brake * cfg.brakeForce + self.inputs.ebrake * cfg.eBrakeForce, cfg.brakeForce);
        local throttle = ( self.inputs.throttle - ( cfg.brakeActsAsReverse and self.inputs.brake or 0 ) ) * cfg.engineForce; -- Include braking power in throttle calculation to allow for reverse

        -- If throttle is less than zero we're actually in reverse so cancel out brakes but take into account ebrake
        if cfg.brakeActsAsReverse and throttle < 0 then
            brake = min( self.inputs.ebrake * cfg.brakeForce * cfg.eBrakeForce, cfg.brakeForce);
        end

        --  Resulting force in local car coordinates.
        --  This is implemented as a RWD car only.
        local tractionForce_cx = throttle - brake * Maths.sign(self.velocity_c.x);
        local tractionForce_cy = 0;

        local dragForce_cx = -cfg.rollResist * self.velocity_c.x - cfg.airResist * self.velocity_c.x * abs(self.velocity_c.x);
        local dragForce_cy = -cfg.rollResist * self.velocity_c.y - cfg.airResist * self.velocity_c.y * abs(self.velocity_c.y);

        -- total force in car coordinates
        local totalForce_cx = dragForce_cx + tractionForce_cx;
        local totalForce_cy = dragForce_cy + tractionForce_cy + cos(self.steerAngle) * frictionForceFront_cy + frictionForceRear_cy;

        -- acceleration along car axes
        self.accel_c.x = totalForce_cx / cfg.mass;  -- forward/reverse accel
        self.accel_c.y = totalForce_cy / cfg.mass;  -- sideways accel

        -- acceleration in world coordinates
        self.accel.x = cs * self.accel_c.x - sn * self.accel_c.y;
        self.accel.y = sn * self.accel_c.x + cs * self.accel_c.y;

        -- update velocity
        self.velocity.x = self.velocity.x + self.accel.x * dt;
        self.velocity.y = self.velocity.y + self.accel.y * dt;

        self.absVel = self.velocity.len();

        -- calculate rotational forces
        local angularTorque = (frictionForceFront_cy + tractionForce_cy) * cfg.cgToFrontAxle - frictionForceRear_cy * cfg.cgToRearAxle;

        --  Sim gets unstable at very slow speeds, so just stop the car
        if abs( self.absVel ) < 0.5 and ( not throttle or throttle == 0 ) then
            self.velocity.x, self.velocity.y, self.absVel = 0, 0, 0
            angularTorque, self.yawRate = 0, 0
        end

        local angularAccel = angularTorque / self.inertia;


        self.yawRate = self.yawRate + angularAccel * dt;
        self.heading = self.heading + ( self.yawRate * self.config.yawFactor ) * dt;

        --  finally we can update position
        self.position.x = self.position.x + self.velocity.x * dt;
        self.position.y = self.position.y + self.velocity.y * dt;


    end

    function self.getSpeed()
        return self.velocity_c.x * 3600 / 1000
    end

    function self.getHeading( degrees )
        return degrees and deg( self.heading ) or self.heading
    end

    function self.getMaxSteer( degrees )
        return degrees and deg( self.config.maxSteer ) or self.config.maxSteer
    end


    -- Return the Controller object
	return self

end

return Controller
