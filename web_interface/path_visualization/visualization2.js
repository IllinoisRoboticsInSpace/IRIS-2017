//Arena dimensions
const X_MIN = -147;
const X_MAX = 147;
const Y_MIN = 0;
const Y_MAX = 738;

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
			  .range([0, height]);

	var y = d3.scaleLinear()
			  .domain([Y_MIN, Y_MAX])
			  .range([0, width]);

	//Scale the data
	for (var i = 0; i < unscaledData.length; i++) {
		scaledData.push([y(unscaledData[i][1]), x(unscaledData[i][0])]);
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
	path.moveTo(scaledData[0][0], scaledData[0][1]);

	for (var i = 1; i < scaledData.length; i++) {
		path.lineTo(scaledData[i][0], scaledData[i][1]);
	};

	//Transition the data that needs to change
	d3.select("path").transition()
	  .attr("d", path.toString())
	  .duration(500);
}

//////////////////////////////////////////////////

function updateConnection(svgElement) {
	var path_url1 = "http://127.0.0.1:8000/J1.json";
	var path_url2 = "http://127.0.0.1:8000/J2.json";
	var url1 = "http://127.0.0.1:8000/iris_path_img.png";
	var url2 = "http://127.0.0.1:8000/iris_path_test.png";
	var url = "http://127.0.0.1:8000/iris_path_img.png";

	if(Math.floor(Math.random()*2) + 1 == 1) {
		path_url = path_url1;
		url = url1;
	}
	else {
		path_url = path_url2;
		url = url2;
	}

	try {
		console.log("Fetching data from the server...")
		parsedData = $.get(PATH_URL, function(rawData) {
			console.log(rawData)
			updatePath(rawData.data, svgElement);
		});

		document.getElementById("navigation-plot").style.backgroundImage = "url(" + OBS_URL + ")";
	}
	catch(err) {
		alert("Failed to connect to the server.");
	}
}

//////////////////////////////////////////////////

//Initialize data and graph upon loading the page
window.onload = function() {
	//Set the height and width variables
	width = document.getElementById("navigation-plot").clientWidth - 40;
	height = document.getElementById("navigation-plot").clientHeight - 40;

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