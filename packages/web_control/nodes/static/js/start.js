function start(){
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/start_cmd")
    xhr.send(null)   
}

function start_cmd(){
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/start_cmd?com=\"" + document.getElementById("input_text").value + "\"")
    xhr.send(null)
}

function kill(){
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/kill")
    xhr.send(null)
}