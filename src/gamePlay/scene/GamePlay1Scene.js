var FLUID_DENSITY = 0.00014;//流体密度
var FLUID_DRAG = 1.0;//流体阻尼
var g_gps = null;
var GamePlay1Scene  = cc.Scene.extend({
	space:null,  //物理世界
	box : null,
	balls:null,//球
	legR:null,
	legR1:null,
	legL:null,
	legL1:null,
	head:null,
    onEnter:function () {
        this._super();
        g_gps = this;
        this.space = new cp.Space();//创建物理
        this.initPhysics();
        this.chainJoint();
//        this.initBoxWithBody5();
        this.scheduleUpdate();
        

        
        this.space.addCollisionHandler( 8, 0,
        		this.collisionBegin.bind(this),
        		null,
        		null,
        		null
        );
        
        cc.eventManager.addListener({
        	event: cc.EventListener.TOUCH_ONE_BY_ONE,
        	swallowTouches: true,//		事件吞噬
        	onTouchBegan: function(touch, event){
        		var target = event.getCurrentTarget();
        		var point = touch.getLocation();
        		if(point.x>GC.w_2){
        			console.log("右");
        			g_gps.legR.applyImpulse(cp.v(50,100), cp.v(0, 0));
        			g_gps.legR1.applyImpulse(cp.v(100,100), cp.v(0, 0));
        		}else{
        			console.log("左");
        			g_gps.legL.applyImpulse(cp.v(-50,100), cp.v(0, 0));
        			g_gps.legL1.applyImpulse(cp.v(-100,100), cp.v(0, 0));
        		}
        		return true;},
        	onTouchMoved: null,
        	onTouchEnded: null,
        	onTouchCancelled : null
        }, this);
       
    },
    collisionBegin : function ( arbiter, space ) {//撞击开始
//    	var shapes = arbiter.getShapes();
//    	var shapeA = shapes[0];
//    	var shapeB = shapes[1];
//    	var body = shapeA.getBody();
//    	
//    	g_gps.doForceBox(g_gps.head);
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

    	var body = this.space.addBody( new cp.Body(mass, cp.momentForPoly(mass, verts, cp.vzero)));
    	body.setPos(cc.p(GC.w_2,GC.h_2)  );

    	var shape = this.space.addShape( new cp.PolyShape(body, verts, cp.vzero));
    	shape.setElasticity(0.5);
    	shape.setFriction(0.7);
    	shape.setCollisionType(7);
    	return body;
    },
  
    //链条
    chainJoint:function(){
    	var v = cp.v;
    	var space = this.space;
    	

    	

    	var addHead = function(pos)
    	{
//    		var radius = 30;
//    		var mass = 0.02;
//    		var body = space.addBody(new cp.Body(mass, cp.momentForCircle(mass, 0, radius, v(0,0))));
//    		body.setPos(cp.v.add(pos, cp.v.add(boxOffset, cp.v(0, 0))));
//
//    		var shape = space.addShape(new cp.CircleShape(body, radius, v(0,0)));
//    		shape.setElasticity(0);
//    		shape.setFriction(0.7);
//    		shape.group = 1; // use a group to keep the car parts from colliding
//    		
//    		return body;
    		
    		
    		var mass = 1;
    		var width = 30;
    		var height = 30;

    		var body = space.addBody(new cp.Body(mass, cp.momentForBox(mass, width, height)));
    		body.setPos(cp.v.add(pos, cp.v.add(boxOffset, cp.v(0, 0))));
    		var shape = space.addShape(new cp.BoxShape(body, width, height));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);
    		shape.setCollisionType(7);	
    		shape.group = 2; // use a group to keep the car parts from colliding
    		return body;
    	};

    	
    	var addChassis1 = function(pos,height)
    	{
    		var mass = 1;
    		var width = 4;

    		var body = space.addBody(new cp.Body(mass, cp.momentForBox(mass, width, height)));
    		body.setPos(cp.v.add(pos, cp.v.add(boxOffset, cp.v(0, 0))));
//    		body.setAngVel(5);// 向心力?
    		var shape = space.addShape(new cp.BoxShape(body, width, height));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);
    		shape.setCollisionType(7);	
    		shape.group = 1; // use a group to keep the car parts from colliding
    		return body;
    	};
    	
    	var addChassis2 = function(pos,height,type)
    	{
    		var mass = 1;
    		var width = 4;

    		var body = space.addBody(new cp.Body(mass, cp.momentForBox(mass, width, height)));
    		body.setPos(cp.v.add(pos, cp.v.add(boxOffset, cp.v(0, 0))));
    		var shape = space.addShape(new cp.BoxShape(body, width, height));
    		shape.setElasticity(0);
    		shape.setFriction(0.7);
    		shape.setCollisionType(type);	
  		     shape.group = 2; // use a group to keep the car parts from colliding
    		return body;
    	};
    	
    	var boxOffset = v(GC.w_2, 220);
    	var staticBody = space.staticBody;
    	
    	var head = addHead(v(0, 0));
    	this.head = head;
    	space.addConstraint(new cp.PivotJoint(head, staticBody, cp.v.add(boxOffset, v(10, 0))));
    	space.addConstraint(new cp.PivotJoint(head, staticBody, cp.v.add(boxOffset, v(-10, 0))));
    	
    	var posC = v(20, -70);
    	legR = addChassis1(posC,100,7);
    	space.addConstraint(new cp.GrooveJoint(head, legR, v(20, -19), v(20, -20), v(0,50)));
    	this.legR = legR;
    	
    	var posC1 = v(20, -170);
    	legR1 = addChassis2(posC1,100,8);
    	this.legR1 = legR1;
    	space.addConstraint(new cp.PivotJoint(legR, legR1, cp.v.add(boxOffset, v(18, -120))));
    	
    	
    	
    	var posB = v(-20, -70);
    	legL = addChassis1(posB,100,7);
    	space.addConstraint(new cp.GrooveJoint(head, legL, v(-20, -19), v(-20, -20), v(0,50)));
    	this.legL = legL;
    	
    	var posB1 = v(-20, -170);
    	legL1 = addChassis2(posB1,100,6);
    	this.legL1 = legL1;
    	space.addConstraint(new cp.PivotJoint(legL, legL1, cp.v.add(boxOffset, v(-18, -120))));

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
    doForceBox: function (body) {
    	body.applyImpulse(cp.v(100,100), cp.v(100, 100));//
    },
    /**
     * 测试用的物理模型
     */
    initDebugMode: function () {
    	this._debugNode = cc.PhysicsDebugNode.create(this.space);
    	this.addChild(this._debugNode);
    }
});