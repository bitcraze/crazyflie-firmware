// Copyright 2011-2013 The MathWorks, Inc.


function local_onload() {
    if (typeof top.rtwreport_document_frame !== "undefined") {
        var docObj = window.document;
        var alink =  docObj.getElementById("linkToText_plain");
        if (alink) {
            alink.href = "matlab:coder.internal.editUrlTextFile('" + alink.href + "')";
        }
        alink = docObj.getElementById("linkToCS");
        if (alink) {
            alink.href = "matlab:coder.internal.viewCodeConfigsetFromReport('" + alink.href + "');";
        }
    }
}

var utils = (function() {

    // Load via Microsoft.XMLDOM--for older versions of IE
    function loadXML_MSXMLDOM(filename, callback, async) {
        if (navigator.appName == "Microsoft Internet Explorer") {
            // Internet Explorer 5/6 
            try {
                var xmlDoc = new ActiveXObject("Microsoft.XMLDOM");
                xmlDoc.async = async;
                xmlDoc.onreadystatechange = function() {
                    if (xmlDoc.readyState == 4) {
                        callback(xmlDoc);
                    }
                }
                xmlDoc.load(filename);
                return true;
            } catch(e) {
            }
        }
        return false;
    }

    // Load via XMLHttpRequest
    function loadXML_XHR(filename, callback, async) {
        if (window.XMLHttpRequest) {
            try {
                var xhr = new XMLHttpRequest();
                xhr.onreadystatechange = function() {
                    if (this.readyState == 4) {
                        callback(this.responseXML);
                    }
                }
                xhr.open("GET", filename, async);
                xhr.send("");
                return true;
            } catch(e) {
                if (navigator.appName === "Netscape" && e.code === 1012) {
                    // file not found: ignore
                    return true;
                }
            }
        }
        return false;
    }

    return {
        trimText: function(s) {
            // In IE9, String.trim not present
            if (s && s.trim) {
                return s.trim();
            }
            else {
                return s;
            }
        },
        getText: function(elt) {
            // In IE9, use 'text' property rather than 'textContent'
            return elt.textContent ? elt.textContent : elt.text;
        },
        loadXML: function(filename, callback, options) {
            var async = !!options && typeof(options["async"]) !== "undefined" ? options.async : true;
            if (!loadXML_XHR(filename, callback, async)) {
                if (!loadXML_MSXMLDOM(filename, callback, async)) {
                    return false;
                }
            }
            return true;
        }
    };
})();

function code2model(sid) {
    utils.loadXML("http://localhost:31415/matlab/feval/coder.internal.code2model?arguments=[\"" + sid + "\"]", function() {});
    //window.location.href = "matlab:coder.internal.code2model('" + sid + "')";
}
