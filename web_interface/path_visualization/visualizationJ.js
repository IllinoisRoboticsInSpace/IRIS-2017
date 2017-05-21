//Arena dimensions
const X_MIN = -80;
const X_MAX = 80;
const Y_MIN = 0;
const Y_MAX = 180;
//const X_MIN = -2;
//const X_MAX = 3;
//const Y_MIN = 0;
//const Y_MAX = 5;

//SVG options
const PATH_COLOR = "#e02828";

//URLs for obstacle map & path data
const OBS_URL = "http://odroid-desktop:8080/M"
const PATH_URL = "http://odroid-desktop:8080/J"
//const OBS_URL = "M"
//const PATH_URL = "J"

//Height & width of div element for scaling SVG
var height = 0;
var width = 0;
const resize_mult=3;

//////////////////////////////////////////////////

//Scales positive and negative data into [0, width], [0, height]
function scaleData(unscaledData, height, width) {
	scaledData = [];
	//Scale the data
	for (var i = 0; i < unscaledData.length; i++) {
		scaledData.push([(unscaledData[i][0]-X_MIN)*width/(X_MAX-X_MIN), (unscaledData[i][1]-Y_MIN)*height/(Y_MAX-Y_MIN),unscaledData[i][3]]);
	};
	return scaledData;
}

//////////////////////////////////////////////////
function unscalePoint(point, height, width) {
	//scale the data
	xcor = Math.round((point[0]*(X_MAX-X_MIN))/width+X_MIN);
	ycor =  Math.round((point[1]*(Y_MAX-Y_MIN))/height+Y_MIN);
	return	[xcor,ycor];
}
//////////////////////////////////////////////////

function updatePath(data_path) {
	//DRAW PATH ******************************
	var one=scaleData([[0,0,0]],height,width)[0];
	var one1=scaleData([[1,1,0]],height,width)[0];
	one=[one1[0]-one[0],one1[1]-one[1]];
	s_data_path = scaleData(data_path, height, width);
	//Set the path element
	var path = d3.path();
	if(scaledData.length != 0) {
		path.moveTo(s_data_path[0][0], s_data_path[0][1]);
		for (var i = 1; i < s_data_path.length; i++) {
			path.lineTo(s_data_path[i][0]+one[0]/2, s_data_path[i][1]+one[1]/2);
		}
	}
	d3.select("#navigationPlot").select("#path_plan").attr("d", path.toString());
}
function updateMap(data_map) {
	//DRAW MAP  ******************************
	for(var x=0;x < (X_MAX-X_MIN);x++) {
		for(var y=0; y<Y_MAX;y++) {
			d3.select("#navigationPlot").select("#map_sq_"+x+"_"+y).style("fill", data_map[x][y]==0?"black":data_map[x][y]==1?"white":"purple");
		}
	}
}
function updatePos(pos) {
	pos=scaleData([pos], height, width)[0];
	var path = d3.path();
	path.moveTo(pos[0], pos[1]);
	var x1=pos[0]+10*Math.sin(pos[2]);
	var y1=pos[1]-10*Math.cos(pos[2]);
	path.lineTo(x1,y1);
	d3.select("#navigationPlot").select("#path_pos_1").attr("d",path.to_string());
	path = d3.path();
	path.moveTo((x1+pos[0]-10*Math.cos(pos[2]))/2., (y1+pos[1]-10*Math.sin(pos[2]))/2.);
	path.lineTo(x1,y1);
	d3.select("#navigationPlot").select("#path_pos_1").attr("d",path.to_string());
	path = d3.path((x1+pos[0]+10*Math.cos(pos[2]))/2., (y1+pos[1]+10*Math.sin(pos[2]))/2.);
	path.moveTo(pos[0], pos[1]);
	path.lineTo(x1,y1);
	d3.select("#navigationPlot").select("#path_pos_1").attr("d",path.to_string());
}

//Initialize the path visualization
function setupPathElements() {
	//Create the SVG element
	var svg = d3.select("#navigationPlot")
				.append("svg")
				.attr("id","svg_plot")
				.attr("width", width)
				.attr("height", height);

	//Create empty map
	var one=scaleData([[0,0,0]],height,width)[0];
	var one1=scaleData([[1,1,0]],height,width)[0];
	one=[one1[0]-one[0],one1[1]-one[1]];

	for(var x=0;x < (X_MAX-X_MIN);x++) {
		for(var y=0; y<Y_MAX;y++) {
			svg .append("rect")
				.attr("id", "map_sq_"+x+"_"+y )
				.attr("x", one[0]*x)
				.attr("y", one[1]*y)
				.attr("width", one[0]*.9)
				.attr("height", one[1]*.9)
				.style("fill", "blue")
				.attr("onclick", "alert(\"position is "+(x+X_MIN)+", "+y+"\")");
		}
	}

	//Plot the path
	svg.append("path")
				  .attr("id", "path_plan")
				  .attr("d", d3.path().toString())
				  .attr("stroke", PATH_COLOR)
				  .attr("stroke-width", 1)
				  .attr("fill", "none");

	//Position arrow
	svg.append("path")
				  .attr("id", "path_pos_1")
				  .attr("d", d3.path().toString())
				  .attr("stroke", PATH_COLOR)
				  .attr("stroke-width", 1)
				  .attr("fill", "none");
	svg.append("path")
				  .attr("id", "path_pos_2")
				  .attr("d", d3.path().toString())
				  .attr("stroke", PATH_COLOR)
				  .attr("stroke-width", 1)
				  .attr("fill", "none");
	svg.append("path")
				  .attr("id", "path_pos_3")
				  .attr("d", d3.path().toString())
				  .attr("stroke", PATH_COLOR)
				  .attr("stroke-width", 1)
				  .attr("fill", "none");
	
	
	return svg;
}

//////////////////////////////////////////////////

function updateConnection(svgElement) {
	try {
		obsData = $.get(OBS_URL, function(rawData) {
			updateMap(JSON.parse(rawData).data);
			updatePos(JSON.parse(rawData).position);
		})
		console.log("Fetching data from the server...");
		parsedData = $.get(PATH_URL, function(rawData) {
			updatePath(JSON.parse(rawData).data);
		});

		//var time = new Date();
		//background = $.get(OBS_URL, function(data) {
			//console.log(data);
		//});
		//document.getElementById("navigationPlot").style.backgroundImage = "url(" + OBS_URL + "?" + time.getTime() + ")";
	}
	catch(err) {

		console.log("Failed to connect to the server.");
	}
}

//////////////////////////////////////////////////

//Initialize data and graph upon loading the page
window.onload = function() {
	//Set the height and width variables
	width = (X_MAX-X_MIN)*resize_mult;
	height = Y_MAX*resize_mult;

	//Get the SVG element for later updating
	var svgElement = setupPathElements();
	
	//Connect to the server to get data
	updateConnection(svgElement);

	//Set the interval timer
	var timer = setInterval(function() {
		updateConnection(svgElement);},
		500);
}
