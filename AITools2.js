var GenericTools;
(function GenericTools){

	var map = (function(){
		function map(array,w,h,t)
		{
			this.dim = t?3:2;
			this.width = w;
			this.height = h;
			this.thickness = t?t:0;
			this.array = array;
		}

		map.prototype.getIndex(i,j,k){
			if(this.dim==2)
			{
				return i*this.width + j;
			}
			else
			{
				return k*this.width*this.height + i*this.width + j;
			}
		}

		map.prototype.getCoordinates(index){
			if(this.dim==2)
			{
				return [(index - index%this.width)/this.height,index%this.width];
			}
			else
			{

			}
		}

		map.prototype.getI(index){
			if(this.dim==2)
			{
				return (index - index%this.width)/this.height;
			}
		}

		map.prototype.getJ(index){
			if(this.dim==2)
			{
				return index%this.width;
			}
		}

		map.prototype.getK(index){
			if(this.dim==2)
			{
				return 
			}
		}
	})
	GenericTools.map = map;

})(GenericTools || (GenericTools={}));

var AITools2;
(function(AITools2){

	var Pathfinder = (function()
	{
		function Pathfinder(config)
		{
			if(!config){
				config = {

				}
			}
		}
		Pathfinder.prototype.solve = function(){

		};
		Pathfinder.prototype.algorithms = {
			Astar : function(start,target,map,distance,heuristicMap,dim){
				if(!dim) dim = 2;
				var closedset = [];
				var openset = [startIndex];
				var GMap = [];
				var FMap = [];
				var GMap[startIndex]=0;
				var FMap[startIndex]=this.heuristicMap[startIndex];
				var cameFrom=[];

				while(this.openset.length>0)
				{
					var THIS=this;
					this.openset.sort(function(a,b){
						return THIS.FMap[b]-THIS.FMap[a];
					});
					var current = this.openset.pop();
					if( current === targetIndex) return this.reconstructPath(this.cameFrom, current);
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
			}
		};
		return Pathfinder;

	});

	AITools2.Pathfinder = Pathfinder;

	})(AITools2 || (AITools2={}));