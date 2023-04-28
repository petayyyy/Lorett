function sendRequestLoop() {
    setInterval(sendRequest, 1000);
}

function sendRequest() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function() {
        document.getElementById("output_text").innerHTML = xhr.response;
    }
    var arg = document.getElementById("input_text").value;
    xhr.open("GET", "/topic_request/" + arg)
    xhr.send(null)
}