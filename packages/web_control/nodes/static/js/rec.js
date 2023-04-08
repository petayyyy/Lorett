const rad = 320;



function updateNumber() {
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (xhr.readyState === XMLHttpRequest.DONE) {
      if (xhr.status === 200) {
        //document.getElementById('number').innerHTML = xhr.responseText;
        var data = JSON.parse(xhr.response);
        const canvasPlot = document.getElementById('canvas_plot');
        const ctx = canvasPlot.getContext('2d');


        ctx.clearRect(0, 0, 700, 700);

        ctx.beginPath();
        ctx.strokeStyle = "#000000";
        
        ctx.arc(350,350,rad,0,Math.PI*2,false);
        
        ctx.stroke();
        ctx.closePath();


        ctx.beginPath();
        ctx.strokeStyle = "#2600ff";
        
        ctx.arc(data.x*rad + canvasPlot.clientWidth/2, data.y*rad*-1 + canvasPlot.clientHeight/2, 10, 0, Math.PI*2, false);
        
        ctx.closePath();
        ctx.stroke();


        ctx.beginPath();
        ctx.strokeStyle = "#ff4500";
        
        ctx.arc(data.cellX*rad + canvasPlot.clientWidth/2, data.cellY*rad*-1 + canvasPlot.clientHeight/2, 5, 0, Math.PI*2, false);
        
        ctx.closePath();
        ctx.stroke();


        document.getElementById('height').innerHTML = "Height: " + data.height;
      }
    }
  };
  xhr.open('GET', '/get_pos');
  xhr.send(null);
}

setInterval(updateNumber, 500);