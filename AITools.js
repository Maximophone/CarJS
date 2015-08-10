var AITools;
(function(AITools){

	var GridMap = (function()
	{
		function GridMap(w,h,initGrid)
		{
			if(!initGrid) initGrid=this.emptyGrid(w,h);
			if(initGrid.length!==w*h) throw "initGrid has invalid size";
			this.grid = initGrid;
			this.width=w;
			this.height = h;
			this.compactGrid = this.compactGrid(this.grid);
			this.clearanceMap = this.computeClearance();

		}
		GridMap.prototype.emptyGrid = function(w,h)
		{
			var grid = [];
			for(var i = 0;i<w*h; ++i) grid.push(false);
			return grid;
		};
		GridMap.prototype.compactGrid = function(grid)
		{
			return 0;
		};
		GridMap.prototype.computeClearance = function()
		{
			return this.grid.map(function(current,index){
				return this.computeClearanceCell(index);
			},this);
		};
		GridMap.prototype.computeClearanceCell = function(i,j)
		{
			var index;
			if(j===undefined){
				index = i;
				i = this.getI(index);
				j = this.getJ(index);
			}
			else
			{
				index = getIndex(i,j);
			}
			var clearance = 0;
			while(true)
			{
				if(clearance >= this.width-j) break; //else we would leave the grid
				if(clearance >= this.height-i) break; //idem
				if(!this.testClearance(clearance+1,i,j)) break;
				clearance++;
			}
			return clearance;
		};
		GridMap.prototype.testClearance = function(clearance,i,j)
		{
			var k,l;
			var clear=1;
			for(k = 0; k<clearance; ++k)
			{
				for(l=0; l<clearance; ++l)
				{
					clear &= !this.grid[this.getIndex(i+k,j+l)];
				}
			}
			return clear;
		};
		GridMap.prototype.getIndex = function(i,j)
		{
			return i*this.width + j;
		};
		GridMap.prototype.getI = function(index)
		{
			return (index - index%this.width)/this.height;
		};
		GridMap.prototype.getJ = function(index)
		{
			return index%this.width;
		};
		GridMap.prototype.getCell = function(i,j,table)
		{
			if(!table) table=this.grid;
			return table[i*this.width + j];
		};
		GridMap.prototype.adaptedMap = function(clearance)
		{
			return this.clearanceMap.map(function(current){
				return current<clearance;
			});
		};
		return GridMap;
	})();
	AITools.GridMap = GridMap;

	var GridSolver = (function(){
		function GridSolver(gridMap,config)
		{
			if(!config){
				config = {
					clearance:1,
					weight_data:0.5,
					weight_smooth:0.2,
					tolerance: 0.0001
				};
			}
			if(!gridMap) throw "GridSolver expects a gridMap";
			this.gridMap = gridMap;
			this.grid = gridMap.adaptedMap(config.clearance);
			this.config=config;
			this.path=[];
			this.vectPath=[];
			this.smoothPath=[];
		}
		GridSolver.prototype.reset = function()
		{
			this.path=[];
			this.vectPath=[];
			this.smoothPath=[];
			this.grid = this.gridMap.adaptedMap(this.config.clearance);
		}
		GridSolver.prototype.solve = function(start,target)
		{
			this.start=start;
			this.target=target;
			this.computeHeuristic(target);
			this.Astar(start,target);
			this.vectPath = this.getVectPath();
			this.smoothPath = this.getSmoothPath(this.config.weight_data,this.config.weight_smooth,this.config.tolerance);
		};
		GridSolver.prototype.getVectPath = function()
		{
			var i,j;
			return this.path.map(function(current){
				i = this.gridMap.getI(current);
				j = this.gridMap.getJ(current);
				return new MathLib.Vect(j+this.config.clearance/2,i+this.config.clearance/2);
			},this);
		};
		GridSolver.prototype.getSmoothPath = function(weight_data, weight_smooth, tolerance)
		{
			if(tolerance===undefined) tolerance = 0.0001;
			if(weight_smooth===undefined) weight_smooth = 0.2;
			if(weight_data===undefined) weight_data = 0.5;
			var path = this.getVectPath();
			var smoothPath = this.getVectPath();
			var err = tolerance;
        	var count = 0;
        
        	while(err>=tolerance & count<200)
        	{
            	err=0;
            	var i;
            	for(i=1;i<this.path.length-1;++i)
            	{

            		var x = path[i].x;
            		var y = path[i].y;
            
            		var x1 = smoothPath[i].x;
            		var y1 = smoothPath[i].y;
            
            		var x2 = smoothPath[i+1].x;
            		var y2 = smoothPath[i+1].y;
            
            		var x0 = smoothPath[i-1].x;
            		var y0 = smoothPath[i-1].y;
            
            		x1 += weight_data * (x-x1);
            		x1 += weight_smooth * (x0 + x2 - 2*x1);
            		err+=Math.abs(smoothPath[i].x-x1);
            		y1 += weight_data * (y-y1);
            		y1 += weight_smooth * (y0 + y2 - 2*y1);
            		err+=Math.abs(smoothPath[i].y-y1);
            		smoothPath[i].x=x1;
            		smoothPath[i].y=y1;
            	}
            count++;
        	}
        	return smoothPath;
		};
		GridSolver.prototype.computeHeuristic = function(target)
		{
			this.heuristicMap = this.grid.map(function(current,index){return this.dist(index,target)},this);
		};
		GridSolver.prototype.Astar = function(start,target)
		{
			this.closedset = [];
			this.openset = [start];
			this.GMap = [];
			this.FMap = [];
			this.GMap[start]=0;
			this.FMap[start]=this.heuristicMap[start];
			this.cameFrom=[];

			while(this.openset.length>0)
			{
				var THIS=this;
				this.openset.sort(function(a,b){
					return THIS.FMap[b]-THIS.FMap[a];
				});
				var current = this.openset.pop();
				if( current === target) return this.reconstructPath(this.cameFrom, current);
				this.closedset.push(current);
				var neighbours = this.getNeighbours(current);
				neighbours.forEach(function(neighbour){
					if(this.closedset.indexOf(neighbour)!==-1) return;
					var tentativeG = this.GMap[current] + this.dist(current,neighbour); //Distance between neighbours
					if(this.openset.indexOf(neighbour)===-1||tentativeG<this.GMap[neighbour])
					{
						this.cameFrom[neighbour]=current;
						this.GMap[neighbour]=tentativeG;
						this.FMap[neighbour]=this.GMap[neighbour]+this.heuristicMap[neighbour];
						if(this.openset.indexOf(neighbour)===-1) this.openset.push(neighbour);
					}
				},this);
			}
			return false;
		};
		GridSolver.prototype.reconstructPath = function(cameFrom,current)
		{
			this.path.push(current);
			if(cameFrom[current]!==undefined)
			{
				this.reconstructPath(cameFrom,cameFrom[current]);
			}
		};
		GridSolver.prototype.getNeighbours = function(index)
		{
			var i=this.gridMap.getI(index);
			var j=this.gridMap.getJ(index);
			var neighbours = [],k,l;
			for(k=-1; k <= 1 ; ++k)
			{
				for(l=-1; l <= 1 ; ++l)
				{
					if(k===0 && l===0) continue;
					if(i+k<0||i+k>=this.gridMap.height||j+l<0||j+l>=this.gridMap.width) continue;
					if(this.grid[this.gridMap.getIndex(i+k,j+l)]) continue;
					neighbours.push(this.gridMap.getIndex(i+k,j+l));
				}
			}
			return neighbours;
		};
		GridSolver.prototype.dist = function(index1,index2)
		{
			return Math.sqrt((this.gridMap.getI(index1)-this.gridMap.getI(index2))*(this.gridMap.getI(index1)-this.gridMap.getI(index2))+(this.gridMap.getJ(index1)-this.gridMap.getJ(index2))*(this.gridMap.getJ(index1)-this.gridMap.getJ(index2)));
		};
		return GridSolver;
	})();
	AITools.GridSolver = GridSolver;

	var GridEditor = (function(){
		function GridEditor(targetnode,config)
		{
			if(!config) config={
				width:32,
				height:32
			};
			if(!targetnode) {
				targetnode = document.createElement('div');
				targetnode.setAttribute("id","target");
				document.querySelector("body").appendChild(targetnode);
				} 
			this.targetnode=targetnode;
			this.config=config;
			this.mousedown=false;
			this.gridMap = new AITools.GridMap(this.config.width,this.config.height);
			this.init();
			this.setupController();
		}
		GridEditor.prototype.init = function()
		{
			this.divEditor = document.createElement('div');
			this.divEditor.setAttribute("id","gridEditor");
			this.table = document.createElement('table');
			this.divEditor.appendChild(this.table);
			this.tablerows = [];
			this.tablecells = [];
			var currentRow;
			for(var i = 0; i<this.config.height; ++i)
			{
				currentRow = document.createElement('tr');
				this.table.appendChild(currentRow);
				this.tablerows.push(currentRow);
				for(var j = 0; j<this.config.width; ++j)
				{
					currentCell = document.createElement('td');
					currentRow.appendChild(currentCell);
					this.tablecells.push(currentCell);
				}
			}
			this.targetnode.appendChild(this.divEditor);
			this.result = document.createElement('div');
			this.result.setAttribute("id","result");
			this.resultText = document.createElement('p');
			this.result.appendChild(this.resultText);
			this.targetnode.appendChild(this.result);
		};
		GridEditor.prototype.render = function()
		{
			this.gridMap.grid.forEach(function(current,index){
				var colour = current?"black":"white";
				this.tablecells[index].style.backgroundColor = colour;
			},this);
		};
		GridEditor.prototype.setupController = function()
		{
			var THIS=this;
			document.addEventListener("keypress",function(e){
				console.log(e.keyCode);
				if(e.keyCode == 112)
				{
					THIS.printGrid();
				}
			});
			document.addEventListener("mousedown",function(){
				THIS.mousedown=true;
			});
			document.addEventListener("mouseup",function(){
				THIS.mousedown=false;
			});
			this.tablecells.forEach(function(current,index){
				current.addEventListener("mouseover",function(){
					if(THIS.mousedown) THIS.setCellTrue(index);
				});
			});
		};
		GridEditor.prototype.printGrid = function()
		{
			this.resultText.innerText = this.gridMap.grid;
		};
		GridEditor.prototype.triggerCell = function(index)
		{
			this.gridMap.grid[index]=!this.gridMap.grid[index];
			this.render();
		};
		GridEditor.prototype.setCellTrue = function(index)
		{
			this.gridMap.grid[index]=true;
			this.render();
		};
		return GridEditor;
	})();
	AITools.GridEditor = GridEditor;

	var GridPainter = (function(){
		function GridPainter(canvas, config, targetnode){
			if(!config) config={};
			if(!targetnode) {
				targetnode = document.createElement('div');
				targetnode.setAttribute("id","target");
				document.querySelector("body").appendChild(targetnode);
				} 
			if(!canvas) {
				canvas = document.createElement('canvas');
				canvas.setAttribute("id","gridCanvas");
				document.querySelector("body").appendChild(canvas);
				} 

			this.canvas = canvas;
			this.width = canvas.width;
			this.height = canvas.height;
			this.context = canvas.getContext("2d");
			this.targetnode=targetnode;
			this.config=config;
			this.initiated=false;
		}
		GridPainter.prototype.init = function(gridMap)
		{
			this.divEditor = document.createElement('div');
			this.divEditor.setAttribute("id","gridEditor");
			this.table = document.createElement('table');
			this.divEditor.appendChild(this.table);
			this.tablerows = [];
			this.tablecells = [];
			var currentRow;
			for(var i = 0; i<gridMap.height; ++i)
			{
				currentRow = document.createElement('tr');
				this.table.appendChild(currentRow);
				this.tablerows.push(currentRow);
				for(var j = 0; j<gridMap.width; ++j)
				{
					currentCell = document.createElement('td');
					currentRow.appendChild(currentCell);
					this.tablecells.push(currentCell);
				}
			}
			this.targetnode.appendChild(this.divEditor);
			this.initiated=true;
		};
		GridPainter.prototype.render = function(grid)
		{
			if(grid.grid) grid = grid.grid;
			if(!this.initiated) this.init(gridMap);
			grid.forEach(function(current,index){
				var colour = current?"black":"white";
				this.tablecells[index].style.backgroundColor = colour;
			},this);
		};
		GridPainter.prototype.renderClearance = function(gridMap)
		{
			if(!this.initiated) this.init(gridMap);
			gridMap.clearanceMap.forEach(function(current,index){
				this.tablecells[index].innerText = current;
			},this);
		};
		GridPainter.prototype.renderHeuristic = function(gridSolver)
		{
			if(!this.initiated) this.init(gridMap);
			gridSolver.heuristicMap.forEach(function(current,index){
				this.tablecells[index].innerText = Math.round(current,0);
			},this);
		};
		GridPainter.prototype.renderPath = function(gridSolver)
		{
			if(!this.initiated) this.init(gridMap);
			gridSolver.path.forEach(function(current){
				this.tablecells[current].style.backgroundColor = "blue";
			},this);
		};
		GridPainter.prototype.drawGrid = function(gridMap)
		{
			gridMap.grid.forEach(function(current,index){
				this.context.fillStyle = current?"black":"white";
				this.context.strokeStyle = current?"black":"white";
				var x = gridMap.getJ(index)*this.width/gridMap.width;
				var y = gridMap.getI(index)*this.height/gridMap.width;
				var w = this.width/gridMap.width;
				var h = this.height/gridMap.width;
				this.context.beginPath();
				this.context.rect(x,y,w,h);
				this.context.fill();
				this.context.stroke();
			},this);
		};
		GridPainter.prototype.drawPath = function(gridSolver,drawClearance)
		{
			gridSolver.path.forEach(function(current){
				this.context.fillStyle = "lightblue";
				this.context.strokeStyle = "lightblue"; 
				var x = gridSolver.gridMap.getJ(current)*this.width/gridSolver.gridMap.width;
				var y = gridSolver.gridMap.getI(current)*this.height/gridSolver.gridMap.width;
				var w = this.width/gridSolver.gridMap.width;
				var h = this.height/gridSolver.gridMap.height;
				this.context.beginPath();
				this.context.rect(x,y,w,h);
				if(drawClearance)
				{
					var i,j;
					for(i = 0; i<gridSolver.config.clearance; ++i)
					{
						for(j = 0; j<gridSolver.config.clearance; ++j)
						{
							this.context.rect(x+i*w,y+j*h,w,h);
						}
					}
				}
				this.context.fill();
				this.context.stroke();
			},this);
		};
		GridPainter.prototype.drawVectPath = function(gridSolver)
		{
			gridSolver.vectPath.forEach(function(current){
				this.context.fillStyle = "red";
				this.context.strokeStyle = "red"; 
				var x = current.x*this.width/gridSolver.gridMap.width;
				var y = current.y*this.height/gridSolver.gridMap.height;
				var R = this.width/gridSolver.gridMap.width/10;
				this.context.beginPath();
				this.context.arc(x, y, R, 0, 2 * Math.PI, false);
				this.context.fill();
				this.context.stroke();
			},this);
		};
		GridPainter.prototype.drawSmoothPath = function(gridSolver)
		{
			gridSolver.smoothPath.forEach(function(current){
				this.context.fillStyle = "green";
				this.context.strokeStyle = "green"; 
				var x = current.x*this.width/gridSolver.gridMap.width;
				var y = current.y*this.height/gridSolver.gridMap.height;
				var R = this.width/gridSolver.gridMap.width/10;
				this.context.beginPath();
				this.context.arc(x, y, R, 0, 2 * Math.PI, false);
				this.context.fill();
				this.context.stroke();
			},this);
		};
		GridPainter.prototype.drawRect = function(i,j,color)
		{
			this.context.fillStyle = color;
			this.context.strokeStyle = color; 
			var x = j*this.width/gridSolver.gridMap.width;
			var y = i*this.height/gridSolver.gridMap.width;
			var w = this.width/gridSolver.gridMap.width;
			var h = this.height/gridSolver.gridMap.height;
			this.context.beginPath();
			this.context.rect(x,y,w,h);
			this.context.fill();
			this.context.stroke();
		}
		return GridPainter;
	})();
	AITools.GridPainter = GridPainter;

	var GridControler = (function()
	{
		function GridControler(painter)
		{
			this.painter = painter;
			this.init();
			this.repeat = 100;
			this.callbacks = {
				mousein:[],
				mousedown:[],
				mousewheel:[]
			};
		}
		GridControler.prototype.mouseIn = function()
		{
			this.callbacks.mousein.forEach(function(current){
				current();
			});

		};
		GridControler.prototype.mouseDown = function()
		{
			this.callbacks.mousedown.forEach(function(current){
				current();
			});

		};
		GridControler.prototype.mouseWheel = function(e)
		{
			this.callbacks.mousewheel.forEach(function(current){
				current(e);
			});

		};
		GridControler.prototype.init = function()
		{
			var timers = {};
			var THIS = this;
			this.mouseX = 0;
			this.mouseY = 0;
			this.mouseInCanvas = false;
			this.painter.canvas.addEventListener("mouseover",function(e){
				// console.log("mouse over canvas");
				document.body.style.overflow = "hidden"
				this.mouseInCanvas=true;
				var key = "mouseincanvas";
				if(!(key in timers)) 
				{
					timers[key]= null;
					THIS.mouseIn();
					timers[key]= setInterval(function(){THIS.mouseIn()}, THIS.repeat);
				}
	
			});
			this.painter.canvas.addEventListener("mouseout",function(e){
				document.body.style.overflow = ""
				this.mouseInCanvas = false;
				// console.log("mouse out of canvas");
				var key = "mouseincanvas";
				if (key in timers) {
            		if (timers[key]!==null)
                		clearInterval(timers[key]);
            		delete timers[key];
        		}
			});
			this.painter.canvas.addEventListener("mousedown",function(e){
				THIS.mouseDown();
			});
			this.painter.canvas.addEventListener("mousewheel",function(e){
				THIS.mouseWheel(e);
			});
			this.painter.canvas.addEventListener("mousemove",function(e){
				//console.log("mouse moved in "+e.clientX);
				var rect = THIS.painter.canvas.getBoundingClientRect();
				THIS.mouseX = e.clientX-rect.left;
				THIS.mouseY = e.clientY-rect.top;
			});
		};
		GridControler.prototype.getMouseI = function(mouseY,height,I)
		{
			return Math.floor(mouseY/height*I);
		};
		GridControler.prototype.getMouseJ = function(mouseX,width,J)
		{
			return Math.floor(mouseX/width*J);
		};
		GridControler.prototype.addPathControls = function(gridSolver){
			this.pathStartState = false;
			this.pathStart;
			this.pathStartI;
			this.pathStartJ;
			this.clearance = gridSolver.config.clearance;
			var THIS = this;
			this.callbacks.mousein.push(function(){
				THIS.painter.drawGrid(gridSolver.gridMap);
				var clearance = gridSolver.config.clearance;
				for(i=0; i<clearance; ++i)
				{
					for(j=0; j<clearance; ++j)
					{
						THIS.painter.drawRect(THIS.pathStartI+i,THIS.pathStartJ+j,"lightblue");
					}
				}
				THIS.painter.drawPath(gridSolver,true);
				THIS.painter.drawVectPath(gridSolver);
				THIS.painter.drawSmoothPath(gridSolver);
				var mouseI = THIS.getMouseI(THIS.mouseY,THIS.painter.height,gridSolver.gridMap.height);
				var mouseJ = THIS.getMouseJ(THIS.mouseX,THIS.painter.width,gridSolver.gridMap.width);
				var i,j;
				var index = gridSolver.gridMap.getIndex(mouseI-Math.floor(THIS.clearance/2),mouseJ-Math.floor(THIS.clearance/2));
				var color = gridSolver.gridMap.clearanceMap[index]>=THIS.clearance?"rgba(0,0,255,0.3)":"rgba(255,0,0,0.3)";
				for(i=0; i<THIS.clearance; ++i)
				{
					for(j=0; j<THIS.clearance; ++j)
					{
						THIS.painter.drawRect(mouseI+i-Math.floor(THIS.clearance/2),mouseJ+j-Math.floor(THIS.clearance/2),color);
					}
				}
			});
			this.callbacks.mousedown.push(function(){
				var clearance = THIS.clearance;
				var mouseI = THIS.getMouseI(THIS.mouseY,THIS.painter.height,gridSolver.gridMap.height);
				var mouseJ = THIS.getMouseJ(THIS.mouseX,THIS.painter.width,gridSolver.gridMap.width);
				if(THIS.pathStartState)
				{
					var pathEnd = gridSolver.gridMap.getIndex(mouseI-Math.floor(clearance/2),mouseJ-Math.floor(clearance/2));
					if(gridSolver.gridMap.clearanceMap[pathEnd]>=clearance)
					{
						THIS.pathStartState = false;
						gridSolver.solve(THIS.pathStart,pathEnd);
					}
				}
				else
				{
					var pathStart = gridSolver.gridMap.getIndex(mouseI-Math.floor(clearance/2),mouseJ-Math.floor(clearance/2));
					if(gridSolver.gridMap.clearanceMap[pathStart]>=clearance)
					{
						THIS.pathStartState = true;
						gridSolver.config.clearance=THIS.clearance;
						gridSolver.reset();
						THIS.pathStart = pathStart;
						THIS.pathStartI = mouseI-Math.floor(clearance/2);
						THIS.pathStartJ = mouseJ-Math.floor(clearance/2);
					}
				}
			});
			this.callbacks.mousewheel.push(function(e){
					var delta = Math.max(-1, Math.min(1, (e.wheelDelta || -e.detail)));
					THIS.clearance+=delta;
					if(THIS.clearance===0) THIS.clearance=1;
			});
		}
		return GridControler;
	})();
	AITools.GridControler = GridControler;


})(AITools || (AITools={}));