#include "Arduino.h"
/*
 * HTML and Javascript code
 */
const char funcbody[] PROGMEM = R"===(
<!DOCTYPE html>
<html><body>

<button type="button" onclick="wallFollowingHit()"> Wall Folowing </button> <br>

<button type="button" onclick="beaconSensingHit()"> Beacon Sensing </button> <br> 

<button typer="button" onclick="moveToPosHit()"> Moving to given Position </button> <br>

<button type="button" onclick="switchmode()"> &nbsp; switch to Manual Mode &nbsp;  </button> <br>

<!-- <input type="range" min="0" max="100" value="50.0" step="1" id="slider2">
<span id="dc">  </span>  <br>
-->

</body>

<script>
  function wallFollowingHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "wallFollowingHit", true);
    xhttp.send();
  }

  function beaconSensingHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "beaconSensingHit", true);
    xhttp.send(); 
  }

  function moveToPosHit() {
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", "moveToPosHit", true);
    xhttp.send();  
  }
  
  setInterval(updateLabel, 50); 
  function updateLabel() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("direction").innerHTML = this.responseText;
      }
    };
    xhttp.open("GET", "MotorDirection", true);
    xhttp.send();
  }
  
  /* slider2.onchange = function() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        document.getElementById("dc").innerHTML = this.responseText;
      }
    };
    var str = "slider2?val=";
    var res = str.concat(this.value);
    xhttp.open("GET", res, true);
    xhttp.send();
  } */

  function switchmode() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        location.reload();
      }
    };
    xhttp.open("GET", "switchmode", true);
    xhttp.send();
  }

</script>

</html>
)===";
