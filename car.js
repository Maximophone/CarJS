var SimCar;
(function(SimCar){
	var CarEngine = (function()
	{
		function CarEngine (config)
		{
			if(config) this.config = config;
			else {
				this.config = {
					gearRatios: [0,2.66,1.78,1.30,1.0,0.74,0.50],
					reverseRatio: 2.90,
					differentialRatio: 3.42,
					transmissionEfficiency: 0.7,
					rpmRange: [1000,6000]
				};
			}
			this.throttle = 0;
			this.gear = 0;
			this.rpm=0;
			this.torque=0;
			this.force=0;
			this.maxForce = 10000;
			this.reverseForce = 5000;
			this.driveTorque = 0; // to divide by radius of wheels to get Fdrive
		}
		CarEngine.prototype.setThrottle = function(th)
		{
			this.throttle = th;
			this.update();
		};
		CarEngine.prototype.setGear = function(gear)
		{
			this.gear = gear;
			this.update();
		};
		CarEngine.prototype.update = function()
		{
			this.force = this.maxForce*this.throttle;
			this.driveTorque = this.torque*this.config.gearRatios[this.gear]*this.config.differentialRatio*this.config.transmissionEfficiency;
		};
		return CarEngine;
	})();

	SimCar.CarEngine = CarEngine;

	var Car = (function(){
		function Car(x,y,mass,world,engine)
		{
			if(!engine) engine = new SimCar.CarEngine();
			if(!world) world = new SimCar.World();
			if(!mass) mass=1500;
			this.engine = engine;
			this.world = world;
			this.mass = mass;
			this.pos = new MathLib.Vect(x,y); //position of the mass center
			this.prevPos = new MathLib.Vect(x,y); 
			this.velocity = new MathLib.Vect(0,0);
			this.acc = new MathLib.Vect(0,0);
			this.angle = Math.PI/6;
			this.angularV = 0;
			this.angularAcc = 0;
			this.forceAppliedRel = new MathLib.Vect(0,0);
			this.torque = 0;
			this.momentumApplied = 0;
			this.wheelAngle = 0;
			this.brakes = {
				amount:0,
				Cbraking:10
			};
			this.geom={
				scale: 1, //general scale of the car
				RO: 0.4, //position of the mass center from the rear wheel
				RW: 0.3, //lateral distance from center to rear wheel
				FW: 0.2, //lateral distance from center to front wheel
				h: 0.3, //height of mass center above the wheels
				WRad: 0.1 //Wheel Radius
			};
			this.inertia = 1500;
			this.frictionCoeff = 2;
			this.corneringStiffnessFront = -5;
			this.corneringStiffnessRear = -5.2;
			this.FBraking=new MathLib.Vect(0,0);
			this.Ftraction=new MathLib.Vect(0,0);
			this.Fdrag=new MathLib.Vect(0,0);
			this.Frr=new MathLib.Vect(0,0);
			this.FlateralFront=new MathLib.Vect(0,0);
			this.FlateralFrontBody=new MathLib.Vect(0,0);
			this.FlateralRear=new MathLib.Vect(0,0);
		}
		Car.prototype.getOrientation = function()
		{
			var mat = MathLib.Mat22.Rotation(this.angle);
			return mat.mulV(new MathLib.Vect(1,0));
		};
		Car.prototype.getRelWheelOrientation = function()
		{
			var mat = MathLib.Mat22.Rotation(this.wheelAngle);
			return mat.mulV(new MathLib.Vect(1,0));
		};
		Car.prototype.getAbsWheelOrientation = function()
		{
			var mat = MathLib.Mat22.Rotation(this.wheelAngle);
			return mat.mulV(this.getOrientation());
		};
		Car.prototype.getRelVelocity = function()
		{
			var mat = MathLib.Mat22.Rotation(-this.angle);
			return mat.mulV(this.velocity);
		};
		Car.prototype.getAbsForceApplied = function()
		{
			var mat = MathLib.Mat22.Rotation(this.angle);
			return mat.mulV(this.forceAppliedRel);
		};
		Car.prototype.addForce = function(force)
		{
			this.forceAppliedRel.addDirect(force);
		};
		Car.prototype.physics = function(dt)
		{
			this.engine.update();
			this.Wr = (1-this.geom.RO)*this.mass*this.world.g + this.geom.h*this.mass*this.acc.dot(this.getOrientation()); 
			this.Wf = this.mass*this.world.g - this.Wr;

			this.fwheelLateralSpeed = this.angularV*(1-this.geom.RO)*this.geom.scale;
			this.rwheelLateralSpeed = this.angularV*this.geom.RO*this.geom.scale;

			var slipAngle = 0;
			var alphaFront = 0;
			var alphaRear = 0;
			if(this.getRelVelocity().x)
			{
				slipAngle = Math.atan2(this.getRelVelocity().y,this.getRelVelocity().x);
				alphaFront = Math.atan2(this.fwheelLateralSpeed,this.getRelVelocity().x);
				alphaRear = Math.atan2(this.rwheelLateralSpeed,this.getRelVelocity().x);
			}

			this.wheelAngleRectified = this.wheelAngle;
			if (slipAngle>Math.PI/2) {
				slipAngle = Math.PI - slipAngle;
				this.wheelAngleRectified = -this.wheelAngle;
			}
			if (alphaFront>Math.PI/2) alphaFront = Math.PI - alphaFront;
			if (alphaRear>Math.PI/2) alphaRear = Math.PI - alphaRear;
			if (slipAngle<-Math.PI/2) { 
				slipAngle = -Math.PI - slipAngle;
				this.wheelAngleRectified = -this.wheelAngle;
			}
			if (alphaFront<-Math.PI/2) alphaFront = -Math.PI - alphaFront;
			if (alphaRear<-Math.PI/2) alphaRear = -Math.PI - alphaRear;




			this.slipAngleFront = (slipAngle + alphaFront - (this.getRelVelocity().y===0?0:this.wheelAngleRectified))%Math.PI;
			this.slipAngleRear = slipAngle - alphaRear;

			this.FlateralFront = new MathLib.Vect(0,this.corneringStiffnessFront*this.slipAngleFront*this.getRelVelocity().len()*this.Wf);
			this.FlateralFront.y = Math.min(this.frictionCoeff*this.Wf, this.FlateralFront.y);
			this.FlateralFront.y = Math.max(-this.frictionCoeff*this.Wf, this.FlateralFront.y);

			this.FlateralRear = new MathLib.Vect(0,this.corneringStiffnessRear*this.slipAngleRear*this.getRelVelocity().len()*this.Wr);
			this.FlateralRear.y = Math.min(this.frictionCoeff*this.Wr, this.FlateralRear.y);
			this.FlateralRear.y = Math.max(-this.frictionCoeff*this.Wr, this.FlateralRear.y);

			this.Ftraction = new MathLib.Vect(this.engine.force,0);
			//var FBraking = new MathSlipLib.Vect(-this.brakes.amount*this.brakes.Cbraking*MathLib.sign(this.getRelVelocity().x),0);
			var fbraking;
			if(this.getRelVelocity().x<0) fbraking = Math.min(this.brakes.amount,this.engine.reverseForce);//REVERSE
			else fbraking = this.brakes.amount*this.brakes.Cbraking;

			this.FBraking = new MathLib.Vect(-fbraking,0);

			//On the body
			this.Fdrag = this.getRelVelocity().scale(-this.getRelVelocity().len()*this.world.Cdrag);
			this.Frr = this.getRelVelocity().scale(-this.world.Crr);

			// var Ftraction = this.getOrientation().scale(this.engine.force);
			// var Fbraking = this.velocity.normalized().scale(-this.brakes.amount*this.brakes.Cbraking);
			// var Fdrag = this.velocity.scale(-this.velocity.len()*this.world.Cdrag);
			// var Frr = this.velocity.scale(-this.world.Crr);

            this.forceAppliedRel.reset(0,0);
            this.torque = 0;
            this.addForce(this.Ftraction);
            this.addForce(this.Fdrag);
            this.addForce(this.Frr);
            this.addForce(this.FBraking);
            this.addForce(this.FlateralRear);

            //convert FlateralFront in body referential
            var mat = MathLib.Mat22.Rotation(this.wheelAngle);
            this.FlateralFrontBody = mat.mulV(this.FlateralFront);

            this.addForce(this.FlateralFrontBody);

            this.torque = (1 - this.geom.RO) * this.geom.scale * this.FlateralFront.y - this.geom.RO * this.geom.scale * this.FlateralRear.y;

			this.acc = this.getAbsForceApplied().scale(1/this.mass);
			this.angularAcc = this.torque / this.inertia;

			this.velocity.addDirect(this.acc.scale(dt));
			this.angularV += this.angularAcc * dt;

			// if(this.velocity.dot(this.getOrientation())<0) {
			// 	this.velocity.reset(0,0); 
			// 	this.acc.reset(0,0);
			// }

			this.pos.addDirect(this.velocity.scale(dt));
			this.angle += this.angularV*dt;
		};
		
		return Car;
	})();

	SimCar.Car = Car;

	var World = (function(){
		function World()
		{
			this.Cdrag = 5;
			this.Crr = 30;
			this.g=9.81;
		}
		return World;
	})();

	SimCar.World = World;

	var Painter = (function(){
		function Painter(context,scale)
		{
			this.context=context;
			this.scale = scale;
			var THIS = this;
			this.gridPattern = function(x,y)
			{
				THIS.context.strokeStyle = "gray";
				var R = 0.05;
				THIS.context.beginPath();
				THIS.context.arc(x*THIS.scale, y*THIS.scale, R*THIS.scale, 0, 2 * Math.PI, false);
				THIS.context.closePath();
				THIS.context.stroke();
			};
		}
		Painter.prototype.drawGrid = function(position,angle)
		{
			var matGrid = [];
			for(var i = -10; i<10;++i)
			{
				for(var j = -10; j<10;++j)
				{
					matGrid.push([2*i,2*j]);
				}
			}
			//var matGrid = [[0,0],[0,2],[0,4],[0,6],[2,0],[2,2],[2,4],[2,6],[4,0],[4,2],[4,4],[4,6]];
			var THIS = this;
			var vect = new MathLib.Vect(0,0);
			var matGridMoved = matGrid.map(function(current){
				vect.reset(current[0],current[1]);
				vect.subDirect(position);
				//return [vect.x%20,vect.y%20];
				return [THIS.moveInt(vect.x%20,20),THIS.moveInt(vect.y%20,20)];
			});
			matGridMoved.forEach(function(current){
				THIS.gridPattern(current[0],current[1]);
			});
		};
		Painter.prototype.moveInt = function(x,MAX)
		{
			return x<0?x+MAX:x;
		};
		Painter.prototype.draw = function(car)
		{
			this.drawGrid(car.pos);
			this.context.strokeStyle = "black";
			var center = new MathLib.Vect(8,6);
			var posF = center.add(car.getOrientation().scale(car.geom.scale*(1-car.geom.RO)));
			var posR = center.add(car.getOrientation().scale(-car.geom.scale*car.geom.RO));
			this.context.beginPath();
			this.context.moveTo(posR.scale(scale).x,posR.scale(scale).y);
            this.context.lineTo(posF.scale(scale).x,posF.scale(scale).y);          
            this.context.closePath();
            this.context.stroke();
            var posRWL = posR.add(car.getOrientation().normal().scale(car.geom.scale*car.geom.RW));
            var posRWR = posR.add(car.getOrientation().normal().scale(-car.geom.scale*car.geom.RW));
            this.context.beginPath();
			this.context.moveTo(posRWR.scale(scale).x,posRWR.scale(scale).y);
            this.context.lineTo(posRWL.scale(scale).x,posRWL.scale(scale).y);          
            this.context.closePath();
            this.context.stroke();
            var posFWL = posF.add(car.getOrientation().normal().scale(car.geom.scale*car.geom.FW));
            var posFWR = posF.add(car.getOrientation().normal().scale(-car.geom.scale*car.geom.FW));
            this.context.beginPath();
			this.context.moveTo(posFWR.scale(scale).x,posFWR.scale(scale).y);
            this.context.lineTo(posFWL.scale(scale).x,posFWL.scale(scale).y);          
            this.context.closePath();
            this.context.stroke();
            var posRWLR = posRWL.add(car.getOrientation().scale(car.geom.scale*car.geom.WRad));
            var posRWLF = posRWL.add(car.getOrientation().scale(-car.geom.scale*car.geom.WRad));
            this.context.beginPath();
			this.context.moveTo(posRWLF.scale(scale).x,posRWLF.scale(scale).y);
            this.context.lineTo(posRWLR.scale(scale).x,posRWLR.scale(scale).y);          
            this.context.closePath();
            this.context.stroke();
            var posRWRR = posRWR.add(car.getOrientation().scale(car.geom.scale*car.geom.WRad));
            var posRWRF = posRWR.add(car.getOrientation().scale(-car.geom.scale*car.geom.WRad));
            this.context.beginPath();
			this.context.moveTo(posRWRF.scale(scale).x,posRWRF.scale(scale).y);
            this.context.lineTo(posRWRR.scale(scale).x,posRWRR.scale(scale).y);          
            this.context.closePath();
            this.context.stroke();
            var posFWLR = posFWL.add(car.getAbsWheelOrientation().scale(car.geom.scale*car.geom.WRad));
            var posFWLF = posFWL.add(car.getAbsWheelOrientation().scale(-car.geom.scale*car.geom.WRad));
            this.context.beginPath();
			this.context.moveTo(posFWLF.scale(scale).x,posFWLF.scale(scale).y);
            this.context.lineTo(posFWLR.scale(scale).x,posFWLR.scale(scale).y);          
            this.context.closePath();
            this.context.stroke();
            var posFWRR = posFWR.add(car.getAbsWheelOrientation().scale(car.geom.scale*car.geom.WRad));
            var posFWRF = posFWR.add(car.getAbsWheelOrientation().scale(-car.geom.scale*car.geom.WRad));
            this.context.beginPath();
			this.context.moveTo(posFWRF.scale(scale).x,posFWRF.scale(scale).y);
            this.context.lineTo(posFWRR.scale(scale).x,posFWRR.scale(scale).y);          
            this.context.closePath();
            this.context.stroke();
		}
		return Painter;
	})();

	SimCar.Painter = Painter;


})(SimCar || (SimCar = {}));

