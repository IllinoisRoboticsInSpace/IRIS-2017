var t=0;
var i = 0;
var myJSON = '{ "x1":1, "x2":3, "x3":2, "y1":1, "y2":2, "y3":3 }';
var myObj = JSON.parse(myJSON);
document.getElementById("demo").innerHTML = myObj.x1;
var svgns = "http://www.w3.org/2000/svg"
// var svgDocument = event.target.ownerDocument;
// var shape = svgDocument.createElementNS(svgns, "circle");
// shape.setAttributeNS(null, "cx", 25);
// shape.setAttributeNS(null, "cy", 25);
// shape.setAttributeNS(null, "r",  20);
// shape.setAttributeNS(null, "fill", "green");
function myFunction() {
	t=t+0.2;
    document.getElementById("map").innerHTML = "<div>TEST</div>";
    temp = document.getElementById("line").innerHTML;
    document.getElementById("line").innerHTML = temp + "<svg xmlns='http://www.w3.org/2000/svg' version='1.1' width='200px' height='200px'><line x1='-100' y1='-100' x2='" + "5000" + "'y2='5000' style='stroke:rgb(0,0,255);stroke-width:2' />Sorry, your browser does not support inline SVG.</svg>";
    if(i==15){
    	i=0;
    	 document.getElementById("line").innerHTML = ''
}
i++;
}
window.setInterval(myFunction, 100);