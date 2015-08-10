var MathLib;
(function(MathLib){   
    var Vect = (function(){     
        function Vect(x,y)
        {
            this.x=x;
            this.y=y;
        }
        Vect.prototype.toString = function(){
            return "(x:" + this.x + " y:" + this.y + ")";
        };
        Vect.prototype.negate = function(){
            return new Vect( -this.x , -this.y );
        };
        Vect.prototype.scale = function(scalar){
            return new Vect( this.x * scalar , this.y * scalar );
        };        
        Vect.prototype.add = function(v)
        {
            return new Vect( this.x + v.x , this.y + v.y );
        };
        Vect.prototype.addDirect = function(v)
        {
            this.x += v.x ;
            this.y += v.y ;
            return this;
        };
        Vect.prototype.subDirect = function(v)
        {
            this.x -= v.x ;
            this.y -= v.y ;
            return this;
        };
        Vect.prototype.sub = function(v)
        {
            return new Vect( this.x - v.x , this.y - v.y );
        };       
        Vect.prototype.reset = function(x,y)
        {
            this.x=x;
            this.y=y;
        };
        Vect.prototype.set = function(v)
        {
            this.x=v.x;
            this.y=v.y;
        };
        Vect.prototype.len = function()
        {
            return Math.sqrt(this.x*this.x + this.y*this.y);
        };
        Vect.prototype.normalize = function()
        {
            this.x=this.len()===0?0:this.x/this.len();
            this.y=this.len()===0?0:this.y/this.len();
        };
        Vect.prototype.normalized = function()
        {
            var x=this.len()===0?0:this.x/this.len();
            var y=this.len()===0?0:this.y/this.len();
            return new MathLib.Vect(x,y);
        };
        Vect.prototype.normal = function()
        {
            return new Vect( this.y , -this.x );
        };
        Vect.prototype.dot = function(v)
        {
            return this.x * v.x + this.y * v.y;
        };
        Vect.prototype.crossProdV = function(v)
        {
            return this.x * v.y - this.y * v.x ;
        };
        Vect.prototype.crossProd = function(s)
        {
            return new Vect( s * this.y , -s * this.x );
        };
        Vect.prototype.copy = function()
        {
            return new Vect( this.x , this.y );
        };
        Vect.Normalize = function(v)
        {
            return new Vect( v.x / v.len() , v.y / v.len() );
        };
        Vect.Add = function(v1,v2)
        {
            return new Vect( v1.x + v2.x , v1.y + v2.y );
        };
        Vect.Sub = function(v1,v2)
        {
            return new Vect( v1.x - v2.x , v1.y - v2.y );
        };
        Vect.getX = function(vectors)     
        {
            var l = vectors.length;
            var v = [];
            for(var i = 0; i<l; i++)
            {
                v.push(vectors[i].x);
            }
            return v;
        };   
        Vect.getY = function(vectors)     
        {
            var l = vectors.length;
            var v = [];
            for(var i = 0; i<l; i++)
            {
                v.push(vectors[i].y);
            }
            return v;
        };   
        Vect.Max = function(vectors,k)
        {
            var l = vectors.length;
            var v = [];
            var m;
            var i;
            if(k==1)
            {                
                m = vectors[0].x;
                for(i = 1; i<l; i++)
                {
                    m=vectors[i].x>m?vectors[i].x:m;
                }
                return m;
            }
            else if (k==2)
            {
                m = vectors[0].y;
                for(i = 1; i<l; i++)
                {
                    m=vectors[i].y>m?vectors[i].y:m;
                }
                return m;
            }
            else
            {
                throw "Call to MathLib.Vect.Max with invalid second argument";
            }
        };
        Vect.Min = function(vectors,k)
        {
            var l = vectors.length;
            var v = [];
            var m;
            var i;
            if(k==1)
            {                
                m = vectors[0].x;
                for(i = 1; i<l; i++)
                {
                    m=vectors[i].x<m?vectors[i].x:m;
                }
                return m;
            }
            else if (k==2)
            {
                m = vectors[0].y;
                for(i = 1; i<l; i++)
                {
                    m=vectors[i].y<m?vectors[i].y:m;
                }
                return m;
            }
            else
            {
                throw "Call to MathLib.Vect.Max with invalid second argument";
            }
        };
        Vect.CrossProdV = function(v1,v2)
        {
            return v1.x * v2.y - v1.y * v2.x ;
        };
        Vect.CrossProd = function(v , s)
        {
            return new Vect( s * v.y , -s * v.x );
        };
        Vect.projectShapeAxis = function(axis,points)
        {
            var minp = points[0].dot(axis);
            var maxp = minp;
            for(var i = 1; i<points.length; i++)
            {
                minp = points[i].dot(axis)<minp?points[i].dot(axis):minp;
                maxp = points[i].dot(axis)>maxp?points[i].dot(axis):maxp;
            }
            return [minp,maxp];
        };
        Vect.Copy = function(v)
        {
        	return new Vect(v.x,v.y);
        };
        return Vect;       
    })();    
    MathLib.Vect=Vect; 
    var Mat22 = (function(){
        function Mat22(m)
        {
            this.m=m;
        }
        Mat22.prototype.col1 = function()
        {
            return new Vect(this.m[0][0],this.m[1][0]);
        };
        Mat22.prototype.col2 = function()
        {
            return new Vect(this.m[0][1],this.m[1][1]);
        };
        Mat22.prototype.row1 = function()
        {
            return new Vect(this.m[0][0],this.m[0][1]);
        };
        Mat22.prototype.row2 = function()
        {
            return new Vect(this.m[1][0],this.m[1][1]);
        };
        Mat22.prototype.mulV = function(v)
        {
            return new Vect(this.m[0][0] * v.x + this.m[0][1] * v.y , this.m[1][0] * v.x + this.m[1][1] * v.y);
        };
        Mat22.Rotation = function(angle)
        {
            var c = Math.cos(angle);
            var s = Math.sin(angle);
            return new Mat22([[c,-s],[s,c]]);
        };
        return Mat22;
    })();
    MathLib.Mat22 = Mat22;
    MathLib.sign = function(number)
    {
        if (number === 0) return number;
        return number>0?1:-1;
    };
})(MathLib || (MathLib = {}));