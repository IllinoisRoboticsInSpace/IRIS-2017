//Test data for svg plot
var testData1 = '{"data":[[-8.003646,220.031708,3.118608],[-16.310484,224.820860,2.118608],[-17.643134,234.549015,1.540903],[-17.344241,244.544548,1.540903],[-17.045348,254.540080,1.540903],[-16.746455,264.535612,1.540903],[-16.447562,274.531144,1.540903],[-16.148669,284.526676,1.540903],[-15.849776,294.522208,1.540903],[-15.550883,304.517741,1.540903],[-15.251990,314.513273,1.540903],[-14.953097,324.508805,1.540903],[-14.654204,334.504337,1.540903],[-14.355311,344.499869,1.540903],[-14.056418,354.495401,1.540903],[-13.757525,364.490933,1.540903],[-13.458632,374.486466,1.540903],[-13.159739,384.481998,1.540903],[-12.860846,394.477530,1.540903],[-12.561953,404.473062,1.540903],[-12.263060,414.468594,1.540903],[-11.964167,424.464126,1.540903],[-11.665274,434.459658,1.540903],[-11.366381,444.455191,1.540903],[-11.067488,454.450723,1.540903],[-10.768595,464.446255,1.540903],[-10.469702,474.441787,1.540903],[-10.170809,484.437319,1.540903],[-9.032663,494.290803,1.127322],[-1.269782,499.919055,0.127322]]}';
var testData2 = '{"data":[[-4.003646,320.031708,3.118608],[-10.310484,24.820860,2.118608],[-36.643134,300.549015,1.540903],[-2.344241,244.544548,1.540903],[17.045348,54.540080,1.540903],[-1.746455,164.535612,1.540903],[-8.447562,124.531144,1.540903],[-16.148669,284.526676,1.540903],[-15.849776,294.522208,1.540903],[-15.550883,304.517741,1.540903],[-15.251990,314.513273,1.540903],[-14.953097,324.508805,1.540903],[-14.654204,334.504337,1.540903],[-14.355311,344.499869,1.540903],[-14.056418,354.495401,1.540903],[-13.757525,364.490933,1.540903],[-13.458632,374.486466,1.540903],[-13.159739,384.481998,1.540903],[-12.860846,394.477530,1.540903],[-12.561953,404.473062,1.540903],[-12.263060,414.468594,1.540903],[-11.964167,424.464126,1.540903],[-11.665274,434.459658,1.540903],[-11.366381,444.455191,1.540903],[-11.067488,454.450723,1.540903],[-10.768595,464.446255,1.540903],[-10.469702,474.441787,1.540903],[-10.170809,484.437319,1.540903],[-9.032663,494.290803,1.127322],[-1.269782,499.919055,0.127322]]}';
var testData3 = '{"data":[[-8.003646,220.031708,3.118608],[-16.310484,224.820860,2.118608],[-17.643134,234.549015,1.540903],[-17.045348,254.540080,1.540903],[-16.746455,264.535612,1.540903],[-16.447562,274.531144,1.540903],[-15.849776,294.522208,1.540903],[-15.251990,314.513273,1.540903],[-14.953097,324.508805,1.540903],[-14.654204,334.504337,1.540903],[-14.355311,344.499869,1.540903],[-14.056418,354.495401,1.540903],[-13.757525,364.490933,1.540903],[-13.458632,374.486466,1.540903],[-13.159739,384.481998,1.540903],[-12.860846,394.477530,1.540903],[-12.561953,404.473062,1.540903],[-12.263060,414.468594,1.540903],[-11.964167,424.464126,1.540903],[-11.665274,434.459658,1.540903],[-11.366381,444.455191,1.540903],[-11.067488,454.450723,1.540903],[-10.768595,464.446255,1.540903],[-10.469702,474.441787,1.540903],[-16.310484,224.820860,2.118608],[-1.269782,499.919055,0.127322]]}';

//Global constants and variables
const pathColor = "#e02828";

var height = 0;
var width = 0;

//////////////////////////////////////////////////

//Scales positive and negative data into [0, width], [0, height]
function scaleData(unscaledData, height, width) {
	scaledData = [];

	//Compute min and max values in data
	var min_x = Number.POSITIVE_INFINITY;
	var min_y = Number.POSITIVE_INFINITY;
	var max_x = Number.NEGATIVE_INFINITY;
	var max_y = Number.NEGATIVE_INFINITY;

	for (var i = 0; i < unscaledData.data.length; i++) {
		//Find the minimum values
		if(unscaledData.data[i][0] < min_x) {
			min_x = unscaledData.data[i][0];
		}
		if(unscaledData.data[i][1] < min_y) {
			min_y = unscaledData.data[i][1];
		}

		//Find the maximum values
		if(unscaledData.data[i][0] > max_x) {
			max_x = unscaledData.data[i][0];
		}
		if(unscaledData.data[i][1] > max_y) {
			max_y = unscaledData.data[i][1];
		}
	};

	//Create the scalers with computed min/max values
	var x = d3.scaleLinear()
			  .domain([min_x, max_x])
			  .range([20, width - 20]);

	var y = d3.scaleLinear()
			  .domain([min_y, max_y])
			  .range([20, height - 20]);

	//Scale the data
	for (var i = 0; i < unscaledData.data.length; i++) {
		scaledData.push([x(unscaledData.data[i][0]), y(unscaledData.data[i][1])]);
	};

	return scaledData;
}

//////////////////////////////////////////////////

//Initialize the path visualization
function setupPathElements(data) {
	//Scale values
	scaledData = scaleData(data, height, width);

	//Create the SVG element
	var svg = d3.select("div#navigation-plot")
				.append("svg")
				.attr("width", width)
				.attr("height", height);
				//.attr("preserveAspectRatio", "xMinYMin meet")
  				//.attr("viewBox", "0 0 " + width.toString() + " " + height.toString());

	//Set the path element
	var path = d3.path();
	path.moveTo(scaledData[0][0], scaledData[0][1]);

	for (var i = 1; i < scaledData.length; i++) {
		path.lineTo(scaledData[i][0], scaledData[i][1]);
	};

	//Plot the path
	var svgPath = svg.append("path")
				  .attr("d", path.toString())
				  .attr("stroke", pathColor)
				  .attr("stroke-width", 1)
				  .attr("fill", "none");

	return svg;
}

//////////////////////////////////////////////////

//Updates the path with new data
function refreshPath(data, svg) {
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
//Miscellaneous functions

function randomTestPath(svgElement) {
	var selection = Math.floor(Math.random()*3) + 1;
	if(selection == 1) {
		var parsedData = JSON.parse(testData1);
	}
	else if(selection == 2) {
		var parsedData = JSON.parse(testData2);
	}
	else {
		var parsedData = JSON.parse(testData3);
	}

	refreshPath(parsedData, svgElement);
}

//////////////////////////////////////////////////

//Initialize data and graph upon loading the page
window.onload = function() {
	var parsedData = JSON.parse(testData1);

	//Set the height and width variables
	width = document.getElementById("navigation-plot").offsetWidth - 50;
	height = document.getElementById("navigation-plot").offsetHeight - 50;

	var svgElement = setupPathElements(parsedData);

	var button = document.getElementById("refresh-path").onclick = function() {
		if(Math.floor(Math.random()*2) + 1 == 1) {
			var parsedData = JSON.parse(testData1);
		}
		else {
			var parsedData = JSON.parse(testData2);
		}

		refreshPath(parsedData, svgElement);
	}

	//Set the interval timer
	var timer = setInterval(function() { randomTestPath(svgElement); }, 3000);
}