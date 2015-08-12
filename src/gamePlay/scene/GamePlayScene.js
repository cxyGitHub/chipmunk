var FLUID_DENSITY = 0.00014;//流体密度
var FLUID_DRAG = 1.0;//流体阻尼
var g_gps = null;
var GamePlayScene  = cc.Scene.extend({
	space:null,  //物理世界
	box : null,
	balls:null,//球
    onEnter:function () {
        this._super();
        g_gps = this;
        this.space = new cp.Space();//创建物理
        this.initPhysics();
        this.initBoxWithBody();
        this.initBoxWithBody2();
        this.initBoxWithBody3();
        this.initBoxWithBody4();
        this.initBoxWithBody5();
        this.initJoint();
        this.chainJoint();
//      this.doForceBox();
        this.scheduleUpdate();
        this.test();
       
    },
    /**
     * 添加碰撞监听事件
     */
    test:function(){
		  	this.space.addCollisionHandler( 7, 0,
		  	this.collisionBegin.bind(this),
		  	this.collisionPre.bind(this),
		  	this.collisionPost.bind(this),
		  	this.collisionSeparate.bind(this)
		  	);
		  	this.space.addCollisionHandler( 7, 1,
		  	this.collisionBegin.bind(this),
		  	this.collisionPre.bind(this),
		  	this.collisionPost.bind(this),
		  	this.collisionSeparate.bind(this)
		  	);
		  	this.space.addCollisionHandler( 7, 2,
		  	this.collisionBegin.bind(this),
		  	this.collisionPre.bind(this),
		  	this.collisionPost.bind(this),
		  	this.collisionSeparate.bind(this)
		  	);
		  	this.space.addCollisionHandler( 7, 3,
		  	this.collisionBegin.bind(this),
		  	this.collisionPre.bind(this),
		  	this.collisionPost.bind(this),
		  	this.collisionSeparate.bind(this)
		  	);
		  	
		  	this.space.addCollisionHandler( 10, 10,
		  			null,
		  			this.collosjoint.bind(this),
		  			this.collosjoint.bind(this),
		  			this.collosjoint.bind(this)
		  	);
		  	
		  	//物体掉入水中监听  5 是水 7物体 
		  	this.space.addCollisionHandler( 5, 7, 
		  			null, 
		  			this.waterPreSolve,
		  			null, 
		  			null);
	
//		  	this.space.removeCollisionHandler( 1, 2 );//移除碰撞检测
    },
    collosjoint:function(){
    	
    },
    collisionBegin : function ( arbiter, space ) {//撞击开始
    	var shapes = arbiter.getShapes();
    	var shapeA = shapes[0];
    	var shapeB = shapes[1];
    	
    	var collTypeA = shapeA.collision_type;
    	var collTypeB = shapeB.collision_type;
//		console.log( 'Collision Type A:' + collTypeA );
//		console.log( 'end Collision Type B:' + collTypeB );
		this.space.addPostStepCallback(function () {
//			console.log("碰撞结束");
		}.bind(this));
		return true;
    },
    collisionPre : function ( arbiter, space ) {//撞击中
    	//console.log('collision pre');
    	return true;
    },
    
    collisionPost : function ( arbiter, space ) {//撞击结束
    	//console.log('collision post');
    },
    
    collisionSeparate : function ( arbiter, space ) {//撞击后分离
    	//console.log('collision separate');
    },
    /**
     * 浮力计算  多边形物体计算浮力
     * @param arb
     * @param space
     * @param ptr
     * @returns {Boolean}
     */
    waterPreSolve : function(arb, space, ptr) {
    	var shapes = arb.getShapes();

    	var water = shapes[0];
    	var poly = shapes[1];
    	var body = poly.getBody();
    	// Get the top of the water sensor bounding box to use as the water level.
    	var level = water.getBB().t;
    	// Clip the polygon against the water level
    	var count = poly.getNumVerts();
    	
    	var clipped = [];

    	var j=count-1;
    	for(var i=0; i<count; i++) {
    		var a = body.local2World( poly.getVert(j));
    		var b = body.local2World( poly.getVert(i));

    		if(a.y < level){
    			clipped.push( a.x );
    			clipped.push( a.y );
    		}

    		var a_level = a.y - level;
    		var b_level = b.y - level;

    		if(a_level*b_level < 0.0){
    			var t = Math.abs(a_level)/(Math.abs(a_level) + Math.abs(b_level));

    			var v = cp.v.lerp(a, b, t);
    			clipped.push(v.x);
    			clipped.push(v.y);
    		}
    		j=i;
    	}

    	// Calculate buoyancy from the clipped polygon area
    	var clippedArea = cp.areaForPoly(clipped);

    	var displacedMass = clippedArea*FLUID_DENSITY;
    	var centroid = cp.centroidForPoly(clipped);
    	var r = cp.v.sub(centroid, body.getPos());

    	var dt = space.getCurrentTimeStep();
    	var g = space.gravity;

    	// Apply the buoyancy force as an impulse.
    	body.applyImpulse( cp.v.mult(g, -displacedMass*dt), r);

    	// Apply linear damping for the fluid drag.
    	var v_centroid = cp.v.add(body.getVel(), cp.v.mult(cp.v.perp(r), body.w));
    	var k = 1; //k_scalar_body(body, r, cp.v.normalize_safe(v_centroid));
    	var damping = clippedArea*FLUID_DRAG*FLUID_DENSITY;
    	var v_coef = Math.exp(-damping*dt*k); // linear drag
    	//  var v_coef = 1.0/(1.0 + damping*dt*cp.v.len(v_centroid)*k); // quadratic drag
    	body.applyImpulse( cp.v.mult(cp.v.sub(cp.v.mult(v_centroid, v_coef), v_centroid), 1.0/k), r);
      
        // Apply angular damping for the fluid drag.
        var w_damping = cp.momentForPoly(FLUID_DRAG*FLUID_DENSITY*clippedArea, clipped, cp.v.neg(body.p));
        body.w *= Math.exp(-w_damping*dt* (1/body.i));

    	return true;
    },
  
    
 
    /**
     * 初始化物理世界
     */
    initPhysics : function(){
    	var space = this.space ;
    	var staticBody = space.staticBody;

    	//开启物体形状测试
    	this.initDebugMode();
    	
    	space.gravity = cp.v(0, -100);      //重力
    	space.sleepTimeThreshold = 0.5;     //休眠临界时间   。不知道干嘛用的？？？
    	space.collisionSlop = 0.5;          //？？？
    	// Walls--物理世界的四个边界
    	var walls = [ new cp.SegmentShape( staticBody, cp.v(0,0), cp.v(GC.w,0), 0 ),				// bottom
    	              new cp.SegmentShape( staticBody, cp.v(0,GC.h), cp.v(GC.w,GC.h), 0),	// top
    	              new cp.SegmentShape( staticBody, cp.v(0,0), cp.v(0,GC.h), 0),				// left
    	              new cp.SegmentShape( staticBody, cp.v(GC.w,0), cp.v(GC.w,GC.h), 0)	// right
    	];
    	for( var i=0; i < walls.length; i++ ) {
    		var shape = walls[i];
    		shape.setElasticity(1);         //弹性
    		shape.setFriction(5);           //摩擦
    		shape.setCollisionType(i);		//当前钢体的类型 
    		space.addShape(shape);
    		shape.setLayers(1);				
    	}
    	
    	//初始化水
    	var shapeWater = space.addShape(new cp.BoxShape2(staticBody,  new cp.BB(0, 0, GC.w, GC.h_2)) );
    	shapeWater.setSensor(true);
    	shapeWater.setCollisionType(5);
    	
    },
   
    /**
     * 这个必须有，物理世界对刚体的处理 实时刷新物理世界内 钢体的状态。
     * @param dt
     */
    update: function (dt) {
        var steps = 3;
        dt /= steps;
        for (var i = 0; i < 3; i++){
            this.space.step(dt);
        }
    },
    /***
     * 初始化一个钢体1
     */
    initBoxWithBody : function(){
    	//物体的定义
    	var mass = 0.2;//质量
    	var boxWidth = 100
    	var boxHeight = 50;
    	
    	var body = new cp.Body(mass, cp.momentForBox(mass, boxWidth, boxHeight) );
    	body.setPos( cc.p(GC.w_2,GC.h) );//初始位置
//    	body.setVel(cp.v(0, 0));//重心? 偏移力
    	body.setAngVel(-20);// 向心力?
    	this.space.addBody( body );
    	
    	var shape = new cp.BoxShape( body, boxWidth, boxHeight);
    	shape.setElasticity( 0.5 );
    	shape.setFriction( 0.3 );
    	shape.setCollisionType(7);
    	shape.setLayers(3);
    	this.space.addShape(shape);
    	
    	
    	
    },  
    
    /***
     * 初始化一个钢体2
     */
    initBoxWithBody2 : function(){
    	//物体的定义
    	var mass = 0.2;//质量
    	var boxWidth = 50
    	var boxHeight = 50;

    	var body = new cp.Body(mass, cp.momentForBox(mass, boxWidth, boxHeight) );
    	body.setPos( cc.p(GC.w_2-100,GC.h) );//初始位置
//  	body.setVel(cp.v(0, 0));//重心? 偏移力
    	body.setAngVel(-20);// 向心力?
    	this.space.addBody( body );

    	var shape = new cp.BoxShape( body, boxWidth, boxHeight);
    	shape.setElasticity( 0.5 );
    	shape.setFriction( 0.3 );
    	shape.setCollisionType(7);
    	shape.setLayers(3);
//    	this.space.addShape(shape);
    	
    },
    
    /***
     * 初始化一个钢体3 球
     */
    initBoxWithBody3 : function(){
    	//物体的定义
    	var radius = 42;
    	var v = cp.v;
    	var mass = 0.2;//质量

    	var body = new cp.Body(mass, cp.momentForCircle(mass, 0, radius, v(0,0)));//质量为1，内径为1，外径为6 ，重心
    	body.setPos( cc.p(GC.w_2+100,GC.h) );//初始位置
//  	body.setVel(cp.v(0, 0));//重心? 偏移力
    	body.setAngVel(-1);// 向心力?
    	this.space.addBody( body );
    	
    	var shape = new cp.CircleShape(body, radius, v(0, 0));
    	shape.setElasticity(0.8);
    	shape.setFriction( 0.3 );
    	shape.setCollisionType(6);
    	shape.setLayers(3);
    	this.space.addShape(shape);

		var v_texture = cc.textureCache.addImage(res.Balls_png);
		var balls = cc.PhysicsSprite.create(v_texture);
		balls.attr({
			scale: 0.1
		});
		
		balls.setBody(body);
		this.addChild(balls,1);
    },
    /**
     * 多边形
     */
    initBoxWithBody4 : function()
    {
    	var size = 20.0;
    	var mass = 0.03;

    	var verts = [
    	             -size,-size,
    	             -size, size,
    	             0, size,
    	             size,-size,
    	             size,-size-20,
    	             size,-size-30
    	             ];

    	var radius = cp.v.len(cp.v(size, size));
    	var pos = this.rand_pos(radius);

    	var body = this.space.addBody( new cp.Body(mass, cp.momentForPoly(mass, verts, cp.vzero)));
    	body.setPos( cp.v.add(pos, cp.v(GC.w_2, GC.h_2) ) );

    	var r = cp.v.len(pos);

    	body.setAngVel(-20);
    	body.setAngle( Math.atan2(pos.y, pos.x));

    	var shape = this.space.addShape( new cp.PolyShape(body, verts, cp.vzero));
    	shape.setElasticity(0.0);
    	shape.setFriction(0.7);
    	shape.setCollisionType(7);
    	
    },
    rand_pos : function(radius)//随机一个初始化位置
    {
    	var v;
    	do {
    		v = cp.v(Math.random()*(640 - 2*radius) - (320 - radius), Math.random()*(480 - 2*radius) - (240 - radius));
    	} while(cp.v.len(v) < 85.0);

    	return v;
    },
    
    /**
     * n边形 等边
     */
    initBoxWithBody5 : function()
    {
    	var size =50.0;
    	var n = 40;
    	var mass = 0.03;
    	var verts = this.getVerts(n,size); //verts 必须逆向旋转 
    	var radius = cp.v.len(cp.v(size, size));
    	var pos = this.rand_pos(radius);

    	var body = this.space.addBody( new cp.Body(mass, cp.momentForPoly(mass, verts, cp.vzero)));
    	body.setPos(cc.p(GC.w_2,GC.h_2)  );

    	var r = cp.v.len(pos);

//    	body.setAngVel(-20);
//    	body.setAngle( Math.atan2(pos.y, pos.x));
    	
    	var shape = this.space.addShape( new cp.PolyShape(body, verts, cp.vzero));
    	shape.setElasticity(0.0);
    	shape.setFriction(0.7);
    	shape.setCollisionType(7);
    	
    	
    	var v_texture = cc.textureCache.addImage(res.Balls_png);
    	var balls = cc.PhysicsSprite.create(v_texture);
    	balls.attr({
    		scale: 0.1
    	});

    	balls.setBody(body);
    	this.addChild(balls,1);
    	
    },
    /**
     * 联合钢体
     */
    initJoint:function(){
    	var space = this.space;
    	var thi = this;
    	var addBall = function(pos)
    	{
    		var radius = 15;
    		var mass = 1;
    		var body = space.addBody(new cp.Body(mass, cp.momentForCircle(mass, 0, radius, v(0,0))));
    		body.setPos(cp.v.add(pos, boxOffset));

    		var shape = space.addShape(new cp.CircleShape(body, radius, v(0,0)));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);

    		return body;
    	};
    	
    	var addLever = function(pos)
    	{
    		var mass = 1;
    		var a = v(0,  15);
    		var b = v(0, -15);

    		var body = space.addBody(new cp.Body(mass, cp.momentForSegment(mass, a, b)));
    		body.setPos(cp.v.add(pos, cp.v.add(boxOffset, cp.v(0, -15))));

    		var shape = space.addShape(new cp.SegmentShape(body, a, b, 5));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);

    		return body;
    	};

    	var addBar = function(pos)
    	{
    		var mass = 2;
    		var a = v(0,  30);
    		var b = v(0, -30);

    		var body = space.addBody(new cp.Body(mass, cp.momentForSegment(mass, a, b)));
    		body.setPos(cp.v.add(pos, boxOffset));

    		var shape = space.addShape(new cp.SegmentShape(body, a, b, 5));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);

    		return body;
    	};
    	

    	var addWheel = function(pos)
    	{
    		var radius = 15;
    		var mass = 0.2;
    		var body = space.addBody(new cp.Body(mass, cp.momentForCircle(mass, 0, radius, v(0,0))));
    		body.setPos(cp.v.add(pos, boxOffset));

    		var shape = space.addShape(new cp.CircleShape(body, radius, v(0,0)));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);
    		shape.group = 1; // use a group to keep the car parts from colliding

    		return body;
    	};

    	var addChassis = function(pos)
    	{
    		var mass = 5;
    		var width = 80;
    		var height = 30;

    		var body = space.addBody(new cp.Body(mass, cp.momentForBox(mass, width, height)));
    		body.setPos(cp.v.add(pos, boxOffset));

    		var shape = space.addShape(new cp.BoxShape(body, width, height));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);
    		shape.group = 1; // use a group to keep the car parts from colliding

    		return body;
    	};
    	
    	var body1, body2;
    	var v = cp.v;
    	var boxOffset = v(160, 0);
    	var posA = v( 50, 60);
    	var posB = v(110, 60);
    	body1 = addBall(posA,4,7);
    	body2 = addBall(posB,4,7);
    	body2.setAngle(Math.PI);
    	var slideJoint = new cp.SlideJoint(body1, body2, v(15,0), v(15,0), 20, 40);
    	space.addConstraint(slideJoint);

    	
    	/**
    	 * 小车
    	 * */
    	
    	boxOffset = v(180, 200);
    	var wheel1 = addWheel(posA);
    	var wheel2 = addWheel(posB);
    	var chassis = addChassis(v(80, 100));

    	space.addConstraint(new cp.GrooveJoint(chassis, wheel1, v(-30, -10), v(-30, -40), v(0,0)));
    	space.addConstraint(new cp.GrooveJoint(chassis, wheel2, v( 30, -10), v( 30, -40), v(0,0)));

    	space.addConstraint(new cp.DampedSpring(chassis, wheel1, v(-30, 0), v(0,0), 50, 20, 10));
    	space.addConstraint(new cp.DampedSpring(chassis, wheel2, v( 30, 0), v(0,0), 50, 20, 10));
    	
    	
    	//旋转的两杆  
    	boxOffset = v(0, 300);
    	var staticBody = space.staticBody;
    	var posA = v( 50, 60);
    	var posB = v(110, 60);

    	var POS_A = function() { return cp.v.add(boxOffset, posA); };
    	var POS_B = function() { return cp.v.add(boxOffset, posB); };
    	
    	body1 = addBar(posA);
    	body2 = addBar(posB);
    	// Add some pin joints to hold the circles in place.
    	space.addConstraint(new cp.PivotJoint(body1, staticBody, POS_A()));
    	space.addConstraint(new cp.PivotJoint(body2, staticBody, POS_B()));
    	// Make them spin at 1/2 revolution per second in relation to each other.
    	var simpleMotor = new cp.SimpleMotor(body1, body2, 10);
    	space.addConstraint(simpleMotor);
    	
    	
    	//静止的两长杠
    	boxOffset = v(GC.w-200, 300);
    	var staticBody = space.staticBody;
    	var posA = v( 50, 60);
    	var posB = v(110, 60);

    	var POS_A = function() { return cp.v.add(boxOffset, posA); };
    	var POS_B = function() { return cp.v.add(boxOffset, posB); };

    	body1 = addBar(posA);
    	body2 = addBar(posB);
    	// Add some pin joints to hold the circles in place.
    	space.addConstraint(new cp.PivotJoint(body1, staticBody, POS_A()));
    	space.addConstraint(new cp.PivotJoint(body2, staticBody, POS_B()));
    	// Make them spin at 1/2 revolution per second in relation to each other.
    	var gearJoint = new cp.GearJoint(body1, body2, 0, 2);
    	space.addConstraint(gearJoint);
    	
    	
    	//旋转的一条杠
    	boxOffset = v(GC.w-400, 300);
    	var staticBody = space.staticBody;
    	var posA = v( 50, 60);

    	var POS_A = function() { return cp.v.add(boxOffset, posA); };

    	body1 = addBar(posA);
    	// Add some pin joints to hold the circles in place.
    	space.addConstraint(new cp.PivotJoint(body1, staticBody, POS_A()));
    	// Make them spin at 1/2 revolution per second in relation to each other.
    	var gearJoint = new cp.GearJoint(body1, body1, 0, 1);
    	space.addConstraint(gearJoint);
    	
    	
    	
    	//不旋转的两端杠
    	boxOffset = v(480, 120);
    	body1 = addLever(posA);
    	body2 = addLever(posB);
    	// Add some pin joints to hold the circles in place.
    	space.addConstraint(new cp.PivotJoint(body1, staticBody, POS_A()));
    	space.addConstraint(new cp.PivotJoint(body2, staticBody, POS_B()));
    	// Ratchet every 90 degrees
    	var ratchet = new cp.RatchetJoint(body1, body2, 0, Math.PI/2);
    	space.addConstraint(ratchet);
    	
    	
    	
    	
    	//不受重力浮力影响的段 斜坡
    	var GRABABLE_MASK_BIT = 1<<31;
    	var NOT_GRABABLE_MASK = ~GRABABLE_MASK_BIT;
    	
    	var ramp = space.addShape(new cp.SegmentShape(space.staticBody, v(100, 100), v(300, 200), 10));
    	ramp.setElasticity(1);
    	ramp.setFriction(1);
//    	ramp.setLayers(NOT_GRABABLE_MASK);
    	
    	
    	
    },
    //链条
    chainJoint:function(){
    	var v = cp.v;
    	var space = this.space;
    	
    	
    	var addBar = function(pos)
    	{
    		var mass = 2;
    		var a = v(0,  30);
    		var b = v(0, -30);

    		var body = space.addBody(new cp.Body(mass, cp.momentForSegment(mass, a, b)));
    		body.setPos(cp.v.add(pos, cp.v.add(boxOffset, cp.v(0, 30))));

    		var shape = space.addShape(new cp.SegmentShape(body, a, b, 5));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);

    		return body;
    	};
    	
     	var addLever = function(pos)
    	{
    		var mass = 1;
    		var a = v(0,  15);
    		var b = v(0, -15);

    		var body = space.addBody(new cp.Body(mass, cp.momentForSegment(mass, a, b)));
    		body.setPos(cp.v.add(pos, cp.v.add(boxOffset, cp.v(0, -15))));

    		var shape = space.addShape(new cp.SegmentShape(body, a, b, 5));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);

    		return body;
    	};
    	
    	var boxOffset = v(GC.w-300, 300);
    	var staticBody = space.staticBody;
    	var posA = v(50, 60);
    	var posB = v(50, 30);
    	var POS_A = function() { return cp.v.add(boxOffset, posA); };
    	var POS_B = function() { return cp.v.add(boxOffset, posB); };
    	body1 = addBar(posA);
    	body2 = addBar(posB);
    	space.addConstraint(new cp.PivotJoint(body1, staticBody, POS_A()));

    	
    	space.addConstraint(new cp.GrooveJoint(body1, body2, v(0, 35), v(0, 30), v(0,30)));

//    	space.addConstraint(new cp.DampedSpring(body1, body2, v(-50, 0), v(0,0), 50, 20, 10));
    	
    },
    getVerts : function(n,r){//n必须大于3 r 大于0
    	if(n<3)n=3;
    	var verts = [];
    	for(var i = n-1 ; i>=0;i--){
    		var angle = 360/n;
    		angle = angle*i;
    		var radian = 2*Math.PI/360*angle;
    		var sin = Math.sin(radian);
    		var cos = Math.cos(radian);
    		verts.push(Math.round(cos*r));
    		verts.push(Math.round(sin*r));
    	}
    	return verts;
    },
    doForceBox: function () {
    	this.box.getBody().setVel(cp.v(0,0));//重心？偏移力
    	this.box.getBody().applyImpulse(cp.v(0,-100), cp.v(0, 0));//
    },
    /**
     * 测试用的物理模型
     */
    initDebugMode: function () {
    	this._debugNode = cc.PhysicsDebugNode.create(this.space);
    	this.addChild(this._debugNode);
    }
});