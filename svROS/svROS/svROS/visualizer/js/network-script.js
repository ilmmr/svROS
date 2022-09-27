var nodes = {{nodes}}
var connections = {{connections}}

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
                'label': 'data(label)', 
                'width': '100px', 'height': 'data(height)', 
                "shape" : "ellipse"
            }
        },

        {
            selector: ':parent',
            css: {
                'text-valign': 'top',
                'text-halign': 'middle',
                'label': 'data(id)',
                'shape': "rectangle",
                'background-color': "#FFFF33",
                'background-opacity': 0.333,
                'border-color': "#FFFF33",
                'border-opacity': 0.5
            }
        },
        
        {
            selector : "edge",
            css : {
                "width" : 2,
                "line-color" : "#666",
                "line-style" : "solid",
                "curve-style": "bezier",
                'source-arrow-color': '#666',
                "source-arrow-shape" : "triangle",
                'target-arrow-color': '#666',
                "target-arrow-shape" : "triangle",
                "label": "data(relation)"
            }
        },
    ],
}));
/* FILL CY with DATA */
var enclaves = [];
nodes.forEach(node => {
    var id =  node['rosname']
    var enclave = node['enclave']
    if (enclave != '/public') {
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
/* CONFIGURATIONS */
// cy.userZoomingEnabled( false );
cy.nodes().forEach(function( n ){ n.data('height', n.width()); });
cy.layout({ name: 'cose-bilkent' }).run();