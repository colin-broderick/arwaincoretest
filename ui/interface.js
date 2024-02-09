let webSocket = new WebSocket('wss://192.168.2.114:8082/echo');
webSocket.onmessage = function(e)
{
    const message = JSON.parse(e.data);

    const elem = document.getElementById("timestamp");
    elem.innerHTML = message.header.time_stamp.seconds + "." + String(message.header.time_stamp.nanoseconds).padStart(9, "0");
    document.getElementById("source_node_id").innerHTML = message.header.Node_ID;

    // Get position information.
    let text = "<pre>";
    for (const entry in message.data.position)
    {
        text += entry + ": ";
        text += (message.data.position[entry].x.toFixed(2)).padStart(7, " ");
        text += ", ";
        text += (message.data.position[entry].y.toFixed(2)).padStart(7, " ");
        text += ", ";
        text += (message.data.position[entry].z.toFixed(2)).padStart(7, " ");
        text += "<br>";
    }
    text += "</pre>";
    document.getElementById("positions").innerHTML = text;

    // Get timed trust information.
    text = "";
    for (const trusting in message.data.trust)
    {
        text += trusting + "<br>";
        for (const trusted in message.data.trust[trusting])
        {
            text += "-- " + trusted + ": ";
            text += message.data.trust[trusting][trusted] + "<br>";
        }
    }
    document.getElementById("trusts").innerHTML = text;

    // Get RSSI information.
    text = "";
    for (const trusting in message.data.rssi)
    {
        text += trusting + "<br>";
        for (const trusted in message.data.rssi[trusting])
        {
            text += "-- " + trusted + ": ";
            text += (message.data.rssi[trusting][trusted]).toFixed(2) + " ";
            text += "(" + message.data.rssi_band[trusting][trusted] + ") ";
            text += ((message.data.range[trusting][trusted]).toFixed(2)).padStart(5, " ") + " ";
            for (let i = 0; i < parseInt(message.data.rssi_band[trusting][trusted]); i++)
            {
                text += "x";
            }
            text += "<br>";
        }
    }
    document.getElementById("rssi").innerHTML = text;
};
webSocket.onclose = function(e)
{
    setTimeout(function() { connect(); }, 1000);
};

function updateSliderText(elem)
{
    elem.nextElementSibling.value = elem.value;
}

function onChangeSpringTighteningFactor(elem)
{
    updateSliderText(elem);
}

function onChangeMinViableRSSI(elem)
{
    updateSliderText(elem);
}

function onChangeMaxExpectedRSSI(elem)
{
    updateSliderText(elem);
}

function onChangePositionUpdateGain(elem)
{
    updateSliderText(elem);
}
