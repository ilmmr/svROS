var nodes_states = {{nodes_states}}
var inbox = {{inbox}}

var cy = (window.cy = cytoscape({
    container: document.getElementById('display_graph'), // container to render in
    elements: [],
    boxSelectionEnabled: false,
    
    style: [ 
        
        {selector: 'node', css: {"text-valign" : "center", "text-halign" : "center", 'border-color': '#adcfe6', 'background-color': '#adcfe6', "border-opacity" : 0.75, 'background-opacity': 0.5, 'label': 'data(label)', 'width': '100px', 'height': 'data(height)', "shape" : "ellipse",
        "border-width" : 3.0}},

        {
            selector: "node[type = 'state']", 
            css: {
                'background-color': '#FF6347',
                'shape': "rectangle",
            }
        },

        {   
            selector : "edge",
            css : {
                "width" : 3,
                "line-style" : "solid",
                "curve-style": "bezier",
                "source-arrow-shape" : "triangle",
                "target-arrow-shape" : "triangle",
                "label": "m = data(value)"
            }
        },

        {
            selector: "edge[trace = 'T1']", 
            css: {
                "line-color" : "#666",
                'source-arrow-color': '#666',
                'target-arrow-color': '#666'
            }
        },

        {
            selector: "edge[trace = 'T2']", 
            css: {
                "line-color" : "#666",
                'source-arrow-color': '#666',
                'target-arrow-color': '#666'
            }
        },
    ],
}));
/* FILL CY with DATA */
/* CONFIGURATIONS */
cy.userZoomingEnabled( false );
cy.nodes().forEach(function( n ){ n.data('height', n.width()); });
cy.layout({ name: 'cose-bilkent' }).run();