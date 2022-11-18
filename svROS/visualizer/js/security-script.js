var nodes = {{nodes}}
var edges = {{edges}}

var cy = (window.cy = cytoscape({
    container: document.getElementById('display_graph'), // container to render in
    elements: [],
    boxSelectionEnabled: false,
    autounselectify: true,
    wheelSensitivity: 0.1,
    
    style: [ 
        
        {
            selector: 'node', 
            css: {
                "text-valign" : "center", 
                "text-halign" : "center", 
                'border-color': '#adcfe6', 
                'background-color': '#adcfe6', 
                "border-opacity" : 0.75, 'background-opacity': 0.5, "border-width" : 2.0,
                'label': 'data(name)', 
                'width': '100px', 'height': 'data(height)', 
                "shape" : "ellipse"
            }
        },

        {
            selector: "node[type = 'object']", 
            css: {
                'shape': "rectangle",
            }
        },

        {   
            selector : "edge",
            css : {
                "font-size" :'12px',
                "width" : 2,
                "line-style" : "solid",
                "curve-style": "bezier",
                "target-arrow-shape" : "triangle",
                "label": 'data(role)',
                "line-color" : "#262626",
                'source-arrow-color': '#262626',
                'target-arrow-color': '#262626'
            }
        },

        {
            selector: "edge[rule = 'Deny']", 
            css: {
                "line-color" : "#FF6347",
                'source-arrow-color': '#FF6347',
                'target-arrow-color': '#FF6347'
            }
        },
    ],
}));
/* FILL CY with DATA */
nodes.forEach(node => {
    cy.add({ group: 'nodes', data: node});
})
edges.forEach(data => {
    cy.add({ group: 'edges', selectable: true, data: data})
})
/* CONFIGURATIONS */
// cy.userZoomingEnabled( false );
cy.nodes().forEach(function( n ){ n.data('height', n.width()); });
cy.layout({ name: 'cose-bilkent' }).run();