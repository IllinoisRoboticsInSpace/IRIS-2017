//Arena dimensions
const X_MIN = -80;
const X_MAX = 80;
const Y_MIN = 0;
const Y_MAX = 180;

//SVG options
const PATH_COLOR = "#e02828";

//URLs for obstacle map & path data
const OBS_URL = "http://odroid-desktop:8080/?"
const PATH_URL = "http://odroid-desktop:8080/J"

//Height & width of div element for scaling SVG
var height = 0;
var width = 0;

//////////////////////////////////////////////////

//Scales positive and negative data into [0, width], [0, height]
function scaleData(unscaledData, height, width) {
	scaledData = [];

	//Create the scalers with computed min/max values
	var x = d3.scaleLinear()
			  .domain([X_MIN, X_MAX])
			  .range([0, width]);

	var y = d3.scaleLinear()
			  .domain([Y_MIN, Y_MAX])
			  .range([0, height]);

	//Scale the data
	for (var i = 0; i < unscaledData.length; i++) {
		scaledData.push([x(unscaledData[i][0]), y(unscaledData[i][1])]);
	};

	return scaledData;
}

//////////////////////////////////////////////////

//Initialize the path visualization
function setupPathElements() {
	//Create the SVG element
	var svg = d3.select("#navigation-plot")
				.append("svg")
				.attr("width", width)
				.attr("height", height);

	//Set the initial path element
	var path = d3.path();

	//Plot the path
	var svgPath = svg.append("path")
				  .attr("d", path.toString())
				  .attr("stroke", PATH_COLOR)
				  .attr("stroke-width", 1)
				  .attr("fill", "none");
	

	return svg;
}

//////////////////////////////////////////////////

//Updates the path with new data
function updatePath(data, svg) {
	//Scale values
	scaledData = scaleData(data, height, width);

	//Set the path element
	var path = d3.path();

	if(scaledData.length != 0) {
		path.moveTo(scaledData[0][0], scaledData[0][1]);

		for (var i = 1; i < scaledData.length; i++) {
			path.lineTo(scaledData[i][0], scaledData[i][1]);
		};
	}

	//Transition the data that needs to change
	d3.select("path").transition()
	  .attr("d", path.toString())
	  .duration(500);
}

//////////////////////////////////////////////////

function updateConnection(svgElement) {
	try {
		console.log("Fetching data from the server...")
		parsedData = $.get(PATH_URL, function(rawData) {
			updatePath(JSON.parse(rawData).data, svgElement);
		});

		var time = new Date();
		document.getElementById("navigation-plot").style.backgroundImage = "url(" + OBS_URL + "?" + time.getTime() + ")";
	}
	catch(err) {

		console.log("Failed to connect to the server.");
	}
}

//////////////////////////////////////////////////

//Initialize data and graph upon loading the page
window.onload = function() {
	//Set the height and width variables
	width = 160*3;
	height = 180*3;
	//width = document.getElementById("navigation-plot").clientWidth - 40;
	//height = document.getElementById("navigation-plot").clientHeight - 40;

	//Get the SVG element for later updating
	var svgElement = setupPathElements();

	//Connect to the server to get data
	updateConnection(svgElement);

	//Bind click event to the button
	document.getElementById("refresh-connection").addEventListener("click", function() {
		console.log("Refreshing connection to ODroid server.");
		updateConnection(svgElement);
	});

	//Set the interval timer
	var timer = setInterval(function() {
		updateConnection(svgElement);},
		3000);
}