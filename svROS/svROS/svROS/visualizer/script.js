var connections = [
    {
        "relation": "/alarm",
        "source": "/multiplexer",
        "target": "/alarm"
    },
    {
        "relation": "/vel_topic",
        "source": "/multiplexer",
        "target": "/turtlesim"
    },
    {
        "relation": "/high_topic",
        "source": "/safety",
        "target": "/multiplexer"
    },
    {
        "relation": "/low_topic",
        "source": "/random",
        "target": "/multiplexer"
    },
    {
        "relation": "/log",
        "source": "/turtlesim",
        "target": "/safety"
    }
]

const options = {
    mode: 'no-cors',
    headers : { 
        'Content-Type': 'application/json',
        'Accept': 'application/json'
       }
};
fetch('/home/luis/.svROS/projects/Project/data/configurations.json', options)
    .then((res) => {
        return res.json();
    })
    .then((data) => console.log(data['nodes']));


var nodes = [{
    "node": "turtle_multiplexer/multiplexer",
    "package": "turtle_multiplexer",
    "namespace": null,
    "rosname": "/multiplexer",
    "enclave": "/enclave",
    "calls": {
        "subscribe": [
            "/low_topic",
            "/high_topic"
        ],
        "advertise": [
            "/alarm",
            "/vel_topic"
        ]
    }
},
{
    "node": "turtle_random/safety",
    "package": "turtle_random",
    "namespace": null,
    "rosname": "/safety",
    "enclave": "/enclave",
    "calls": {
        "subscribe": [
            "/log"
        ],
        "advertise": [
            "/high_topic"
        ]
    }
},
{
    "node": "turtle_random/random",
    "package": "turtle_random",
    "namespace": null,
    "rosname": "/random",
    "enclave": "",
    "calls": {
        "subscribe": [],
        "advertise": [
            "/low_topic"
        ]
    }
},
{
    "node": "alarm/alarm",
    "package": "alarm",
    "namespace": null,
    "rosname": "/alarm",
    "enclave": "",
    "calls": {
        "subscribe": [
            "/alarm"
        ],
        "advertise": [
            "/light"
        ]
    }
},
{
    "node": "turtlesim/turtlesim",
    "package": "turtlesim",
    "namespace": null,
    "rosname": "/turtlesim",
    "enclave": "/enclave",
    "calls": {
        "subscribe": [
            "/vel_topic"
        ],
        "advertise": [
            "/log"
        ]
    }
}]


var cy = (window.cy = cytoscape({
    container: document.getElementById('display_graph'), // container to render in
    elements: [],
    boxSelectionEnabled: false,
    
    style: [ 
        
        {selector: 'node', css: {"text-valign" : "center", "text-halign" : "center", 'border-color': '#adcfe6', 'background-color': '#adcfe6', "border-opacity" : 0.75, 'background-opacity': 0.5, 'label': 'data(label)', 'width': '100px', 'height': 'data(height)', "shape" : "ellipse",
        "border-width" : 3.0}},

        {
            selector: ':parent',
                css: {
                  'shape': "rectangle",
                  'background-color': "#FFFF33",
                  'background-opacity': 0.333,
                  'border-color': "#FFFF33",
                  'border-opacity': 0.5
                }
            },
        
        {selector : "edge",
        css : {
          "width" : 3,
          "line-color" : "#666",
          "line-style" : "solid",
          "curve-style": "bezier",
          'source-arrow-color': '#666',
          "source-arrow-shape" : "triangle",
          'target-arrow-color': '#666',
          "target-arrow-shape" : "triangle",
          "label": "data(relation)"
        }},
    ],
}));
/* FILL CY DATA */
var already = 0;
var enclaves = [];
if (already == 0) {
    nodes.forEach(node => {
        var id =  node['rosname']
        var enclave = node['enclave']
        if (enclave != '') {
            if (!enclaves.includes(enclave)) {
                cy.add({ group: 'nodes', data: { id: enclave }});
                enclaves.push(node['enclave'])
            }
            cy.add({ group: 'nodes', data: { id: id, label : node['rosname'], parent: node['enclave']}});
        }
        else {
            cy.add({ group: 'nodes', data: { id: id, label : node['rosname'], height: node.width, parent: id}});
        }
    })
    connections.forEach(data => {
        cy.add({ group: 'edges', selectable: true, data: data})
    })
    already = 1; 
}
console.log(enclaves)
/* CONFIGURATIONS */
cy.userZoomingEnabled( false );
cy.nodes().forEach(function( n ){ n.data('height', n.width()); });
cy.layout({ name: 'cose-bilkent' }).run();