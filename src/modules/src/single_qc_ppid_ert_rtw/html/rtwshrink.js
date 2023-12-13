// Copyright 2011-2017 The MathWorks, Inc.

function RTW_STRUCT(prop, value) {
    this.prop = prop;
    if (typeof(value) == 'undefined') {
        this.value = "";
    } else {
        this.value = value;
    }
}

// initialize the cache when code generation report is first loaded
function RTW_BOOK() {
    this.length = 0;
    this.rtw_pages = new Array();
    this.getPage = function(file) {
        return this.rtw_pages[file];
    };
    this.addPage = function(file) {
        var page;
        if (this.hasPage(file)) {
            page = this.rtw_pages[file];
        } else {
            page = new RTW_PAGE(file);
            this.rtw_pages[file] = page;
        }
        return page;
    };
    this.hasPage = function(file) {
        return typeof(this.rtw_pages[file]) != 'undefined';
    };
    this.removePage = function(file) {
            var tmp;
            if (typeof(this.rtw_pages[file]) != 'undefined') {
                tmp = this.rtw_pages[file];
                delete this.rtw_pages[file];
                this.length--;
            }
            return tmp;
        };
}

if (!RTW_BOOK.instance) {
    RTW_BOOK.instance = new RTW_BOOK();
}

function RTW_PAGE() {
    this.length = 0;
    this.items = new Array();
    this.pagename = '';
    if (arguments.length > 0 && typeof(arguments[1]) != 'undefined') {
        this.pagename = arguments[1];
    }

    this.getItem = function(id) {
        return this.items[id];
    };
    this.getItems = function() {
        return this.items;
    };
    this.addItem = function(id, value) {
        var tmp;
        if (typeof(value) != 'undefined') {
            if (typeof(this.items[id]) != 'undefined') {
                this.length++;
            } else {
                tmp = this.items[id];
            }
            this.items[id] = value;
            this.length++;
        }
        return tmp;
    };
    this.hasItem = function(id) {
        return typeof(this.items[id]) != 'undefined';
    };
    this.removeItem = function(id) {
        var tmp;
        if (typeof(this.items[id]) != 'undefined') {
            tmp = this.items[id];
            delete this.items[id];
            this.length--;
        }
        return tmp;
    };
}

function rtwTableShrink(doc, obj, id, isSymbol) {
    var hide, hide_text, show, show_text;
    if (isSymbol) {
        hide = "[-]";
        hide_text = hide;
        show = "[+]";
        show_text = show;
    } else {
        hide = "[<u>hide</u>]";
        hide_text  = "[hide]";
        show = "[<u>show</u>]";
        show_text = "[show]";
    }
    hide = "<span class='shrink-button'>" + hide + "</span>";
    show = "<span class='shrink-button'>" + show + "</span>";
    if (doc.getElementsByName) {
        var o = doc.getElementsByName(id);
        for (var oid = 0; oid < o.length; ++oid) {
            if (o[oid].style.display === "none") {
                o[oid].style.display =  "";
            } else {
                o[oid].style.display = "none";
            }
        }
        if (o.length >= 0 && addToPage) {
            addToPage(doc, o[0], 'display');
        }
    }

    // IE supports innerText while other browsers support textContent
    if (obj.textContent) {
        var objText = obj.textContent;
    } else {
        var objText = obj.innerText;
    }

    if (objText.toLowerCase() === show_text.toLowerCase()) {
        obj.innerHTML = hide;
    } else {
        obj.innerHTML = show;
    }

    if (addToPage) {
        addToPage(doc, obj, 'innerHTML');
    }
}

function rtwTableExpand(doc, controlObj, id) {
    if (doc.getElementById) {
        var obj = doc.getElementById(id);
        if (obj && obj.style.display === "none") {
            rtwTableShrink(doc, controlObj, id, false);
        }
    }
}

function restoreState(docObj) {
    var filename = docObj.location.href;
    if (RTW_BOOK.instance && RTW_BOOK.instance.hasPage(filename)) {
        var page = RTW_BOOK.instance.getPage(filename);
        var items = page.getItems();
        var elem;
        if (docObj.getElementsByName) {
            for (var i in items) {
                var o = docObj.getElementsByName(i);
                for (var oid = 0; oid < o.length; ++oid) {
                    elem = o[oid];
                    if (items[i].prop === 'display') {
                        if (elem.style.display === 'none') {
                            elem.style.display = '';
                        } else {
                            elem.style.display = 'none';
                        }
                    } else if (items[i].prop === 'innerHTML') {
                        elem.innerHTML = items[i].value;
                    }
                }
            }
        }
    }
}

function addToPage(docObj, obj, prop) {
    var filename = docObj.location.href;
    if (RTW_BOOK.instance) {
        var page;
        if (RTW_BOOK.instance.hasPage(filename)) {
            page = RTW_BOOK.instance.getPage(filename);
        } else {
            page = RTW_BOOK.instance.addPage(filename);
        }
        if (page.hasItem(obj.id)) {
            page.removeItem(obj.id);
        } else {
            var my_struct;
            if (prop === "display") {
                my_struct = new RTW_STRUCT(prop, obj.style.display);
            } else {
                my_struct = new RTW_STRUCT(prop, obj.innerHTML);
            }
            page.addItem(obj.id, my_struct);
        }
    }
}

function rtwSwitchView(doc, obj1, obj2) {
    if (doc.getElementsByName) {
        var o = doc.getElementsByName(obj1);
        for (var oid = 0; oid < o.length; ++oid) {
            o[oid].style.display =  "none";
        }
        if (o.length >= 0 && addToPage) {
            addToPage(doc, o[0], 'display');
        }
        var o = doc.getElementsByName(obj2);
        for (var oid = 0; oid < o.length; ++oid) {
            o[oid].style.display =  "";
        }
        if (o.length >= 0 && addToPage) {
            addToPage(doc, o[0], 'display');
        }
    }
}
