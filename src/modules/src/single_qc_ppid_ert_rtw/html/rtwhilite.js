// Copyright 2006-2019 The MathWorks, Inc.

// Class RTW_Hash ------------------------------------------------------------
// Internal web browser doesn't change window.location.hash if the link points
// to the same page.
// RTW_Hash remembers the hash value when the page is loaded in the first time 
// or a link is clicked.
// removeHiliteByHash cleans the high lighted elements according to the stored 
// hash value
function RTW_Hash(aHash) {
    if (aHash == null) {
        this.fHash = "";
    } else {
        this.fHash = aHash;
    };
    
    this.getHash = function() {
        return this.fHash;
    }

    this.setHash = function(aHash) {
        this.fHash = aHash;
    }
}

RTW_Hash.instance = null;

// Class RTW_TraceInfo --------------------------------------------------------
function RTW_TraceInfo(aFileLinks) {
    this.fFileLinks = aFileLinks;
    this.fLines = new Array();
    this.fTotalLines = 0; // total number of highlighted lines
    this.fNumLines = new Array();
    this.fFileIdxCache = new Array();
    this.fDisablePanel = false;
    this.fCurrFileIdx = -1;
    this.fCurrLineIdx = -1;
    this.fCurrCodeNode = null;
    this.getHtmlFileName = function(aIndex) {
        if (aIndex < this.fFileLinks.length) {
            var href = this.fFileLinks[aIndex].href;
            return href.substring(href.lastIndexOf('/')+1);
        }
    }
    this.getSrcFileName = function(aIndex) {
        var name = this.getHtmlFileName(aIndex);
        if (name)
            name = RTW_TraceInfo.toSrcFileName(name);
        return name;
    }
    this.getNumFileLinks = function() {
        return this.fFileLinks.length;
    }
    this.setFileLinkColor = function(aIndex, aColor) {
        var link = this.fFileLinks[aIndex];
        if (link && link.parentNode && link.parentNode.style)
            link.parentNode.style.backgroundColor = aColor;
    }
    this.highlightFileLink = function(aIndex, aColor) {
        for (var i = 0; i < this.fFileLinks.length; ++i) {
            this.setFileLinkColor(i, i == aIndex ? aColor : "");
        }
    }
    this.highlightCurrFileLink = function(aColor) {
        this.highlightFileLink(this.fCurrFileIdx);
    }
    this.highlightLines = function(aCodeNode,aColor) {
        this.fCurrCodeNode = aCodeNode;
        var lines = this.fLines[this.getHtmlFileName(this.fCurrFileIdx)];
        if (lines && aCodeNode) {
            for (var i = 0; i < lines.length; ++i) {
                var lineObj = aCodeNode.childNodes[lines[i]-1];
                if (lineObj)
                    lineObj.style.backgroundColor=aColor;
            }
        }
    }
    this.getFileIdx = function(aFile) {
        if (this.fFileIdxCache[aFile] != null)
            return this.fFileIdxCache[aFile];
        for (var i = 0; i < this.fFileLinks.length; ++i) {
            if (this.getHtmlFileName(i) == aFile) {
                this.fFileIdxCache[aFile] = i;
                return i;
            }
        }
        return null;
    }
    this.getCurrFileIdx = function() { return this.fCurrFileIdx; }
    this.setNumHighlightedLines = function(aFileIdx, aNumLines) {
        this.fNumLines[aFileIdx] = aNumLines;
        updateNumHighlightedLines(this.fFileLinks[aFileIdx], aNumLines);
    }
    this.getNumLines = function(aFileIdx) {
        return this.fNumLines[aFileIdx] != null ? this.fNumLines[aFileIdx] : 0;
    }
    this.getNumLinesAll = function() {
        var sum = 0;
        var len = this.fNumLines.length;
        for (var i = 0; i < len; ++i) {
            sum += this.getNumLines(i);
        }
        return sum;
    }
    this.getPrevButton = function() {
        var aFrame = getNavFrame();
        if (typeof aFrame !== "undefined" && aFrame !== null)
            return aFrame.document.getElementById("rtwIdButtonPrev");
        else
            return document.getElementById("rtwIdButtonPrev");
    }
    this.getNextButton = function() {
        var aFrame = getNavFrame();
        if (typeof aFrame !== "undefined" && aFrame !== null)
            return aFrame.document.getElementById("rtwIdButtonNext");
        else
            return document.getElementById("rtwIdButtonNext");
    }
    this.getPanel = function() {
        var aFrame = getNavFrame();
        if (typeof aFrame !== "undefined" && aFrame !== null)
            return aFrame.document.getElementById("rtwIdTracePanel");
        else
            return document.getElementById("rtwIdTracePanel");
    }
    this.removeHighlighting = function() {
        for (var i = 0; i < this.fFileLinks.length; ++i) {
            this.setFileLinkColor(i, "");
            this.setNumHighlightedLines(i, 0);
        }
        // remove highlight and reset current code node
        try {
            if (this.fCurrCodeNode != null)
                this.highlightLines(getCodeNode(),"");
        } catch (e) {};
        this.fCurrCodeNode = null;    
        if (this.getPrevButton()) { this.getPrevButton().disabled = true; }
        if (this.getNextButton()) { this.getNextButton().disabled = true; }
        if (this.getPanel()) { this.getPanel().style.display = "none"; }
        this.fCurrFileIdx = -1;
        this.fCurrLineIdx = -1;
    }
    this.setCurrLineIdx = function(aLineIdx) {
        this.fCurrLineIdx = aLineIdx;
    }
    this.getCurrLineIdx = function() { return this.fCurrLineIdx; }
    this.setCurrent = function(aFileIdx, aLineIdx) {
        this.fCurrFileIdx = aFileIdx;
        var numLines = this.getNumLines(aFileIdx);
        if (!numLines || aLineIdx >= numLines)
            this.fCurrLineIdx = -1;
        else
            this.fCurrLineIdx = aLineIdx;
        var allNumLines = this.getNumLinesAll();
        if (this.getPrevButton()) {
            this.getPrevButton().disabled = (allNumLines <= 1 || !this.hasPrev());
        }
        if (this.getNextButton()) {
            this.getNextButton().disabled = (allNumLines <= 1 || !this.hasNext());
        }
        if (this.getPanel() && !this.fDisablePanel) {
            this.getPanel().style.display = 'block';
        }
    }
    this.setDisablePanel = function(aDisable) {
        this.fDisablePanel = aDisable;
    }
    this.getPrevFileIdx = function() {
        if (this.fCurrLineIdx > 0)
            return this.fCurrFileIdx;
        for (var i = this.fCurrFileIdx - 1; i >= 0; --i)
            if (this.fNumLines[i] > 0)
                return i;
        return null;
    }
    // update the navigation bar state
    this.updateNavState = function() {
        if (this.getPrevButton())
            this.getPrevButton().disabled = !this.hasPrev();
        if (this.getNextButton())
            this.getNextButton().disabled = !this.hasNext();
        setTraceNumber();
    }
    this.hasPrev = function() {
        return this.getPrevFileIdx() != null;
    }
    this.getFirstFileIdx = function() {
        for (var i = 0; i < this.getNumFileLinks(); ++i)
            if (this.fNumLines[i] > 0)
                return i;
    }
    this.getLastFileIdx = function() {
        for (var i = this.getNumFileLinks(); i >= 0; --i)
            if (this.fNumLines[i] > 0)
                return i;
    }
    this.goFirst = function() {
        this.fCurrFileIdx = this.getFirstFileIdx();
        this.fCurrLineIdx = 0;
        this.updateNavState();
    }
    this.goLast = function() {
        this.fCurrFileIdx = this.getLastFileIdx();;
        this.fCurrLineIdx = this.getNumLines(this.fCurrFileIdx) - 1;
        this.updateNavState();
    }
    this.goPrev = function() {
        var fileIdx = this.getPrevFileIdx();
        if (fileIdx == null)
            return;
        if (fileIdx == this.fCurrFileIdx)
            --this.fCurrLineIdx;
        else {
            this.fCurrFileIdx = fileIdx;
            this.fCurrLineIdx = this.getNumLines(fileIdx) - 1;
        }
        this.updateNavState();
    }
    this.getNextFileIdx = function() {
        if (this.fCurrLineIdx < this.getNumLines(this.fCurrFileIdx) - 1 && this.getNumLines(this.fCurrFileIdx) > 0)
            return this.fCurrFileIdx;
        for (var i = this.fCurrFileIdx + 1; i < this.getNumFileLinks(); ++i)
            if (this.fNumLines[i] > 0)
                return i;
        return null;
    }
    this.hasNext = function() {
        return this.getNextFileIdx() != null;
    }
    this.goNext = function() {
        var fileIdx = this.getNextFileIdx();
        if (fileIdx == null)
            return;
        if (fileIdx == this.fCurrFileIdx)
            ++this.fCurrLineIdx;
        else {
            this.fCurrFileIdx = fileIdx;
            this.fCurrLineIdx = 0;
        }
        this.updateNavState();
    }
    this.setTotalLines = function(num) {
        this.fTotalLines = num;
    }
    this.getTotalLines = function() { return this.fTotalLines;}
    this.setLines = function(aFile, aLines) {
        this.fLines[aFile] = aLines;
        var index = this.getFileIdx(aFile);
        if (index != null)
            this.setNumHighlightedLines(index,aLines.length);
    }
    this.getLines = function(aFile) {
        return this.fLines[aFile];
    }
    // get current on focus line number
    this.getCurrLine = function() {
        var file = this.getHtmlFileName(this.getCurrFileIdx());
        var lines = this.fLines[file];
        var line = null;
        if (lines) {
            var line = lines[this.fCurrLineIdx];
        }
        return line;
    }
    this.getHRef = function(aFileIdx, aLineIdx, offset) {
        var file = this.getHtmlFileName(aFileIdx);
        var lines = this.fLines[file];
        if (lines) {
            var line = lines[aLineIdx];
            line = offset_line(line, offset);
            file = file+"#"+line;
        }
        return file;
    }
    
    this.getCurrentHRef = function(offset) {
        return this.getHRef(this.fCurrFileIdx, this.fCurrLineIdx, offset);
    }
    this.setInitLocation = function(aFile, aLine) {
        var fileIdx = this.getFileIdx(aFile);
        var lineIdx = null;
        if (fileIdx != null && aLine) {
            var lines = this.getLines(aFile);
            for (var i = 0; i < lines.length; ++i) {
                if (lines[i] == aLine) {
                    lineIdx = i;
                    break;
                } 
            }
        }
        if (fileIdx == null || lineIdx == null)
            this.setCurrent(-1,-1);
        else
            this.setCurrent(fileIdx,lineIdx);
    }
}

// Static methods in RTW_TraceInfo

RTW_TraceInfo.getFileLinks = function(docObj) {
    var links;
    if (docObj && docObj.getElementsByName)
        links = docObj.getElementsByName("rtwIdGenFileLinks");
    return links ? links : new Array();
}

RTW_TraceInfo.toSrcFileName = function(aHtmlFileName) {
    aHtmlFileName = aHtmlFileName.replace(/_c.html$/,".c");
    aHtmlFileName = aHtmlFileName.replace(/_h.html$/,".h");
    aHtmlFileName = aHtmlFileName.replace(/_cpp.html$/,".cpp");
    aHtmlFileName = aHtmlFileName.replace(/_hpp.html$/,".hpp");
    aHtmlFileName = aHtmlFileName.replace(/_cc.html$/,".hpp");
    return aHtmlFileName;
}

RTW_TraceInfo.instance = null;

// Class RTW_TraceArgs --------------------------------------------------------
// file.c:10,20,30&file.h:10,20,30[&color=value] or 
// sid=model:1[&color=value]
RTW_TraceArgs = function(aHash) {
    this.fColor = null;
    this.fFontSize = null;
    this.fInitFile = null;
    this.fInitLine = null;
    this.fSID = null;
    this.fFiles = new Array();
    this.fLines = new Array();
    this.fMessage = null;
    this.fBlock = null;  
    this.fNumBlocks = 0;
    this.fUseExternalBrowser = true;
    this.fInStudio = false;
    this.fModel2CodeSrc = null;
    this.fInCodeTrace = false;
    this.fTraceData = null;
    this.fFileIdx = []; // filename to fileId
    this.fRows = []; // highlighted rows indexed by fileId
    this.fIDs = []; // highlighted IDs indexed by fileId

    this.hasSid = function() {
        return !(this.fSID == null);
    }
    this.parseCommand = function(aHash) {
        var args = new Array();
        args = aHash.split('&');
        for (var i = 0; i < args.length; ++i) {
            var arg = args[i];
            sep = arg.indexOf('=');
            if (sep != -1) {
                var cmd = arg.substring(0,sep);
                var opt = arg.substring(sep+1);
                switch (cmd.toLowerCase()) {
                case "color":
                    this.fColor = opt;
                    break;
                case "fontsize":
                    this.fFontSize = opt;
                    break;
                case "initfile":
                    this.fInitFile = RTW_TraceArgs.toHtmlFileName(opt);
                    break;
                case "initline":
                    this.fInitLine = opt;
                    break;
                case "msg":
                    this.fMessage = opt;
                    break;
                case "block":
                    this.fBlock = unescape(opt);
                    break;
                case "numblocks":
                    this.fNumBlocks = parseInt(opt);
                    break;
                case "sid":
                    this.fSID = opt;
                    // convert sid to code location
                    break;
                case "model2code_src":
                    // model2code_src from model or webview
                    this.fModel2CodeSrc = opt;
                    break;
                case "useexternalbrowser":
                    this.fUseExternalBrowser = (opt=="true");
                    break;
                case "instudio":
                    this.fInStudio = (opt=="true");
                    break;
                case "incodetrace":
                    this.fInCodeTrace = (opt=="true");
                    break;
                case "tracedata":
                    this.fTraceData = decodeURI(opt);
                    break;
                }
            }
        }    
    }
    this.parseUrlHash = function(aHash) {
        var rows, sep, assignSep;
        if (aHash) {
            args = aHash.split('&');
            for (var i = 0; i < args.length; ++i) {
                var arg = args[i];
                sep = arg.indexOf(':');
                assignSep = arg.indexOf('=');
                if (sep !== -1 && assignSep === -1) {
                    var fileLines = arg.split(':');
                    var htmlFileName = RTW_TraceArgs.toHtmlFileName(fileLines[0]);
                    this.fFileIdx[htmlFileName] = i;
                    this.fFiles.push(htmlFileName);
                    if (fileLines[1]) {
                        rows = fileLines[1].split(',');
                        rows = uniqueRows(rows);
                        this.fLines.push(rows);
                        this.fRows[i] = rows;
                    }
                }
            }
            if (this.fInitFile == null && this.fFiles.length > 0) {
                this.fInitFile = this.fFiles[0];
                this.fInitLine = (this.fLines[0] == null ? -1 : this.fLines[0][0]);
            }
        }
    }
    this.parseUrlHash2 = function(aHash) {
        aHash = decodeURI(aHash);    
        var rows;
        var ids;
        if (aHash && aHash.length > 0 && aHash[0] === "[") {
            var input = eval(aHash);  
            var i;
            var j;
            // set highlight files from url
            for (i=0; i<input.length;i++) {
                rows = new Array();
                ids = new Array();
                this.fFileIdx[input[i].file] = i;
                this.fFiles.push(input[i].file);
                ids = input[i].id;
                for (j=0; j<ids.length;j++) {
                    // get row number
                    if (ids[j].indexOf("c") !== -1)
                        rows.push(Number(ids[j].substring(0,ids[j].indexOf("c")))); 
                    else
                        rows.push(Number(ids[j]));
                }
                rows = uniqueRows(rows);
                this.fRows[i] = rows;
                this.fIDs[i] = ids;
            }
        } else {
            // reset all states
            this.fFiles = [];
            this.fRows = [];
            this.fIDs = [];
        }
        return;
    }
    this.getFileIdx = function(aFileName) {
        if (aFileName) {
            return this.fFileIdx[aFileName];
        } else {
            // return the fileIdx of the current display file
            var pathname = top.rtwreport_document_frame.location.pathname;
            pathname = pathname.substr(pathname.lastIndexOf("/")+1);
            // find the highlight file name
            return this.getFileIdx(pathname);
        }
    }

    this.getColor = function() { return this.fColor; }
    this.getFontSize = function() { return this.fFontSize; }
    this.getInitFile = function() { return this.fInitFile; }
    this.getInitLine = function() { return this.fInitLine; }
    this.getNumFiles = function() { return this.fFiles.length; }
    this.getSID = function() { return this.fSID; }
    this.getFile = function(aIdx) { if (isNaN(aIdx)) return this.fFiles; return this.fFiles[aIdx];}
    this.getLines = function(aIdx) { return this.fLines[aIdx]; } 
    this.getUseExternalBrowser = function() { return this.fUseExternalBrowser; } 
    this.getInStudio = function() { return this.fInStudio; } 
    this.getInCodeTrace = function() { return this.fInCodeTrace; } 
    this.getTraceData = function() { return this.fTraceData; } 
    this.getModel2CodeSrc = function() { return this.fModel2CodeSrc; }
    this.setUseExternalBrowser = function(val) { this.fUseExternalBrowser = val; } 
    this.setInCodeTrace = function(val) { this.fInCodeTrace = val; } 
    this.setTraceData = function(val) { this.fTraceData = val; } 
    this.setModel2CodeSrc = function(val) { this.fModel2CodeSrc = val; }
    this.getRows = function(aIdx) { return this.fRows[aIdx];}
    this.getIDs = function(aIdx) { return this.fIDs[aIdx]; }
    this.getBlock = function() { return this.fBlock; }
    this.getNumBlocks = function() { return this.fNumBlocks; }
    // constructor
    this.parseCommand(aHash);
}

// Static methods in RTW_TraceArgs

RTW_TraceArgs.toHtmlFileName = function(aFile) {
    f = aFile;
    aFile = f.substring(0,f.lastIndexOf('.')) + '_' + f.substring(f.lastIndexOf('.')+1) + ".html";
    return aFile;
}

RTW_TraceArgs.instance = null;

RTW_MessageWindow = function(aWindow, aParagraph) {
    this.fWindow    = aWindow;
    this.fParagraph = aParagraph;
    
    this.print = function(msg) {
        this.fParagraph.innerHTML = msg;
        if (msg)
            this.fWindow.style.display = "block";
        else
            this.fWindow.style.display = "none";
    }
    this.clear = function() {
        this.print("");
    }
}

// RTW_MessageWindow factory
RTW_MessageWindowFactory = function(aDocObj) {
    this.fDocObj = aDocObj;
    this.fInstance = null;

    this.getInstance = function() {
        if (this.fInstance)
            return this.fInstance;
        if (!this.fDocObj)
            return;
        
        var table     = this.fDocObj.getElementById("rtwIdMsgWindow");
        var paragraph = this.fDocObj.getElementById("rtwIdMsg");
        var button    = this.fDocObj.getElementById("rtwIdButtonMsg");

        if (!table || !paragraph || !button)
            return null;

        obj = new RTW_MessageWindow(table,paragraph);
        button.onclick = function() { obj.clear(); }
        this.fInstance = obj;
        return this.fInstance;
    }
}

RTW_MessageWindowFactory.instance = null;
RTW_MessageWindow.factory = function(aDocObj) {
    if (!RTW_MessageWindowFactory.instance)
        RTW_MessageWindowFactory.instance = new RTW_MessageWindowFactory(aDocObj);
    return RTW_MessageWindowFactory.instance.getInstance();
}

// Callbacks and helper functions ---------------------------------------------

// Helper functions
function getCodeNode() {
    return rtwSrcFrame().document.getElementById("RTWcode");
}

function rtwMidFrame() {
    return top.document.getElementById('rtw_midFrame');
}
function rtwSrcFrame() {
    return top.rtwreport_document_frame;
}
function rtwTocFrame() {
    return top.rtwreport_contents_frame;
}
function rtwNavToolbarFrame() {
    return top.rtwreport_navToolbar_frame; // return rtwTocFrame();
}
function rtwInspectFrame() {
    return top.rtwreport_inspect_frame; // return rtwTocFrame();
}
function rtwGetFileName(url) {
    var slashIdx = url.lastIndexOf('/');
    var hashIdx  = url.indexOf('#', slashIdx);
    if (hashIdx == -1)
        return url.substring(slashIdx+1)
    else
        return url.substring(slashIdx+1,hashIdx);
}

// Help function to expand the file group
function expandFileGroup(docObj, tagID) {
    if (docObj.getElementById) {
        var obj_table = docObj.getElementById(tagID);
        var o;
        while (obj_table.nodeName != "TABLE") {
            if (obj_table.parentNode) {
                obj_table = obj_table.parentNode;
            } else {
                return;
            }
        }
        if (obj_table.style.display == "none") {
            var category_table = obj_table.parentNode;
            while (category_table.nodeName != "TABLE") {
                if (category_table.parentNode) {
                    category_table = category_table.parentNode;
                } else {
                    return;
                }        
            }
            var o = category_table.id + "_button";
            o = docObj.getElementById(o);
            if (o && top.rtwreport_contents_frame.rtwFileListShrink) {
                top.rtwreport_contents_frame.rtwFileListShrink(o, category_table.id,
                                                               category_table.getAttribute('label'), 0);
            }
        }
    }
}
// Help function to set the background color based on Element's Id in a document
// object
function setBGColorByElementId(docObj, tagID, bgColor) {
    var status = false;
    if (bgColor == "") {
        bgColor = "TRANSPARENT";
    }
    
    if (docObj.getElementById) {
        var obj2Hilite = docObj.getElementById(tagID);
        if (obj2Hilite && obj2Hilite.parentNode) {
            obj2Hilite.parentNode.style.backgroundColor = bgColor;
            status = true;
        }
    }
    return status;
}

// Help function to set the background color based on Element's name in a document
// object
function setBGColorByElementsName(docObj, tagName, bgColor) {
    if (bgColor == "") {
        bgColor = "TRANSPARENT";
    }  
    if (docObj.getElementsByName) {
        var objs2Hilite = docObj.getElementsByName(tagName);
        for (var objIndex = 0; objIndex < objs2Hilite.length; ++objIndex) {     
            if (objs2Hilite[objIndex].parentNode)
                objs2Hilite[objIndex].parentNode.style.backgroundColor = bgColor;
        }
    }
}

// Help function to highlight lines in source file based on Element's name
// Note: Name of docHiliteByElementsName would be better
function hiliteByElementsName(winObj, tagName) {
    var hiliteColor = "#aaffff";
    if (winObj.document)
        setBGColorByElementsName(winObj.document, tagName, hiliteColor);
}

// Help function to remove the highlight of lines in source file based on Element's name
function removeHiliteByElementsName(winObj, tagName) {
    if (winObj.document)
        setBGColorByElementsName(winObj.document, tagName, "");
}

// Help function to set the background color based on the URL's hash
function setBGColorByHash(docObj, bgColor) {    
    if (docObj.location) {
        var tagName = docObj.location.hash;
        // Use the stored hash value if it exists because the location.hash
        // may be wrong in internal web browser
        if (RTW_Hash.instance)
            tagName = RTW_Hash.instance.getHash();
        if (tagName != null)
            tagName = tagName.substring(1);
        
        var codeNode = docObj.getElementById("RTWcode");
        if (tagName != null && tagName != "") {        
            if (!isNaN(tagName))
                tagName = Number(tagName) + 10;            
            setBGColorByElementsName(docObj, tagName, bgColor);
        }
    }
}

// Highlight the lines in document frame based on the URL's hash
function hiliteByHash(docObj) {       
    var hiliteColor = "#aaffff";  
    setBGColorByHash(docObj, hiliteColor);
}

// Remove highlight of lines in document frame based on the URL's hash
function removeHiliteByHash(winObj) {
    if (winObj.document)
        setBGColorByHash(winObj.document, "");
}

// Highlight the filename Element in TOC frame based on the URL's filename
function hiliteByFileName(aHref) {       
    var status = false;
    if (!top.rtwreport_contents_frame)
        return status;
    var hiliteColor = GlobalConfig.fileLinkHiliteColor;
    var fileName = rtwGetFileName(aHref);    
    if (top.rtwreport_contents_frame.document) {
        removeHiliteFileList(top.rtwreport_contents_frame);
        status = setBGColorByElementId(top.rtwreport_contents_frame.document, fileName, hiliteColor);
        if (status)
            expandFileGroup(top.rtwreport_contents_frame.document, fileName);
    }
    return status;
}

// Clear the highlights in the code navigation frame.
function removeHiliteCodeNav(winObj) {    
    removeHiliteTOC(winObj);
    removeHiliteFileList(winObj);
}
// Clear the highlights in TOC frame. TOC links are named TOC_List
function removeHiliteTOC(winObj) {    
    removeHiliteByElementsName(winObj, "TOC_List"); 
}
// Clear the highlights in Generated File List. 
// The filename links are named rtwIdGenFileLinks,
function removeHiliteFileList(winObj) {    
    removeHiliteByElementsName(winObj, "rtwIdGenFileLinks");
}

// Highlight TOC hyperlinks by their Ids.
function tocHiliteById(id) {
    hiliteColor = GlobalConfig.fileLinkHiliteColor;    
    if (top && top.rtwreport_contents_frame && top.rtwreport_contents_frame.document) {
        removeHiliteCodeNav(top.rtwreport_contents_frame);
        setBGColorByElementId(top.rtwreport_contents_frame.document, id, hiliteColor);
    }
}

// onClick function to highlight the link itself
function tocHiliteMe(winObj, linkObj, bCleanTrace) {
    hiliteColor = GlobalConfig.fileLinkHiliteColor;
    // remove the trace info (previous highlighted source code and the navigate
    // panel)
    // Clean Trace info only when links in TOC clicked. Links of filenames won't
    // clean trace info. 
    if (bCleanTrace) {
        if (RTW_TraceInfo.instance) {
            RTW_TraceInfo.instance.setDisablePanel(true);
            rtwRemoveHighlighting();
        }
        closeInspectWindow();        
    }        
    removeHiliteCodeNav(winObj);
    if (linkObj.parentNode) {
        linkObj.parentNode.style.backgroundColor= hiliteColor;
    }
}

// onClick function to clean the currently highlighed lines in document frame
// based on URL's hash
// Then highlight lines in document frame based on Element's name
// It works for links to some elements in the same page, otherwise, 
// rtwFileOnLoad() in loading page does the job.
function docHiliteMe(winObj, elementName) {
    // First, remove the highlighted elements by stored hash value
    removeHiliteByHash(winObj);
    // Store the new hash value defined by elementName
    if (RTW_Hash.instance) {
        RTW_Hash.instance.setHash("#"+elementName);
    } else {
        RTW_Hash.instance = new RTW_Hash("#"+elementName);
    }
    hiliteByElementsName(winObj, elementName);
}

// Callback for generated file load callback
function rtwFileOnLoad(docObj) {
    if (!docObj.location || !docObj.location.href)
        return;
    // Save the hash value when file is loaded in the first time
    if (!RTW_Hash.instance) {
        RTW_Hash.instance = new RTW_Hash(docObj.location.hash);
    } else {
        RTW_Hash.instance.setHash(docObj.location.hash);
    }  
    
    updateHyperlinks();
    // highlight lines in source code file according to the URL hash
    hiliteByHash(docObj);
    // highlight the filename in the TOC frame
    if (top.rtwreport_contents_frame) {
        if (hiliteByFileName(docObj.location.href)) {
            // remove the highlights in the TOC frame if filename is hilite successfully
            removeHiliteTOC(top.rtwreport_contents_frame);
        }
    }
    
    if (!RTW_TraceInfo.instance)
        return;
    if (!docObj.getElementById)
        return;
    if (rtwSrcFrame())
        rtwSrcFrame().focus();
    var fileName = rtwGetFileName(docObj.location.href);
    var fileIdx = RTW_TraceInfo.instance.getFileIdx(fileName);
    if (fileIdx != null) {
        if (fileIdx != RTW_TraceInfo.instance.getCurrFileIdx())
            RTW_TraceInfo.instance.setCurrent(fileIdx,-1);
        var codeNode = docObj.getElementById("RTWcode");
        var hiliteColor = RTW_TraceArgs.instance.getColor();
        if (!hiliteColor) {
            hiliteColor = "#aaffff";
        }
        var fontSize = RTW_TraceArgs.instance.getFontSize();
        if (fontSize) {
            codeNode.style.fontSize = fontSize;
        }
        RTW_TraceInfo.instance.highlightLines(codeNode,hiliteColor);
        RTW_TraceInfo.instance.highlightFileLink(fileIdx, GlobalConfig.fileLinkHiliteColor);
    }
}

function Nav(fileIdx1, fileIdx2) {
    var filename = top.rtwreport_document_frame.location.pathname.split(/\//);
    filename = filename[filename.length-1];
    var currentFileIdx = RTW_TraceInfo.instance.getFileIdx(filename);
    if (fileIdx1 === currentFileIdx) {
        top.rtwreport_document_frame.document.location.href = RTW_TraceInfo.instance.getCurrentHRef();
        top.initLine = top.rtwreport_document_frame.document.location.hash.substr(1);
        addTagToCurrentLine();
        if (top.rtwreport_contents_frame) {            
            if (hiliteByFileName(top.rtwreport_document_frame.location.href))
                removeHiliteTOC(top.rtwreport_contents_frame);
        }
    } else {
        var aUrl = RTW_TraceInfo.instance.getCurrentHRef();
        if (hasWebviewFrame()) {
            top.rtwreport_document_frame.document.location.href=aUrl;
        } else {
            top.rtwreport_document_frame.document.location.href=aUrl + "+newPage";
        }
    }
}
// Callback for "Prev" button
function rtwGoPrev() {
    if (RTW_TraceInfo.instance && top.rtwreport_document_frame) {
        var prevfileIdx = RTW_TraceInfo.instance.getPrevFileIdx();
        var currfileIdx = RTW_TraceInfo.instance.fCurrFileIdx;
        rmTagToCurrentLine();
        RTW_TraceInfo.instance.goPrev();
        Nav(prevfileIdx, currfileIdx);
    }
}
// Callback for "First" button
function rtwGoFirst() {
    if (RTW_TraceInfo.instance && top.rtwreport_document_frame) {
        var prevfileIdx = RTW_TraceInfo.instance.getFirstFileIdx();
        var currfileIdx = RTW_TraceInfo.instance.fCurrFileIdx;
        rmTagToCurrentLine();
        RTW_TraceInfo.instance.goFirst();
        Nav(prevfileIdx, currfileIdx);
    }
}

// Callback for navigation button onclick
var navButtonStatus = (function() {
    var isclicked = false;
    return {
        clicked: function () {
            isclicked = true;
            return false;
        },
        reset: function () {
            isclicked = false;
        },
        isClicked: function () {
            return isclicked;
        }
    };
})();

// Callback for "Next" button
function rtwGoNext() {
    if (RTW_TraceInfo.instance && top.rtwreport_document_frame) {
        var nextfileIdx = RTW_TraceInfo.instance.getNextFileIdx();
        var currfileIdx = RTW_TraceInfo.instance.fCurrFileIdx;
        rmTagToCurrentLine();
        RTW_TraceInfo.instance.goNext();
        Nav(nextfileIdx, currfileIdx);
    }
}
// Callback for "Last" button
function rtwGoLast() {
    if (RTW_TraceInfo.instance && top.rtwreport_document_frame) {
        var nextfileIdx = RTW_TraceInfo.instance.getLastFileIdx();
        var currfileIdx = RTW_TraceInfo.instance.fCurrFileIdx;
        rmTagToCurrentLine();
        RTW_TraceInfo.instance.goLast();
        Nav(nextfileIdx, currfileIdx);
    }
}

function addTagToCurrentLine() {
    rmHiliteClickedToken();
    tagCurrentLine(true);
}
function rmTagToCurrentLine() {
    tagCurrentLine(false);
}
// tag current line by changing the bacgkround color of the line 
function tagCurrentLine(addColor) {
    if (RTW_TraceInfo.instance) {
        var o = top.rtwreport_document_frame.document.getElementById(RTW_TraceInfo.instance.getCurrLine());
        if (o) {
            if (addColor) {
                o.className = "hiliteCurrentLine";            
            } else {
                o.className = "hilite";
            }
        }
    }
}
// Helper function for main document load callback
function rtwMainOnLoadFcn(topDocObj,aLoc,aPanel,forceReload) {
    var loc;
    var aHash="";
    var lastArgs = null;
    var tocDocObj = top.rtwreport_contents_frame.document;
    if (typeof forceReload === "undefined") {
        forceReload = false;
    }
    // get the hash value from location.
    if (!aLoc) {
        loc = topDocObj.location;
        if (loc.search || loc.hash) {
            if (loc.search)
                aHash = loc.search.substring(1);
            else
                aHash = loc.hash.substring(1);
        }
    } else {
        aHash = aLoc;
        if (RTW_TraceArgs.instance)
            lastArgs = RTW_TraceArgs.instance;
    }

    // parse URL hash value
    RTW_TraceArgs.instance = new RTW_TraceArgs(aHash);
    // load metrics
    load_metrics();
    // hide content panel if in studio
    if (RTW_TraceArgs.instance.getInStudio()) {
        setupInStudio();
    }
    // use incode traceability
    if (RTW_TraceArgs.instance.getInCodeTrace()) {
        RTW_TraceArgs.instance.parseUrlHash2(RTW_TraceArgs.instance.getTraceData());
        inCodeTraceOnload();
        return;
    }
    if (lastArgs !== null) {
        RTW_TraceArgs.instance.setUseExternalBrowser(lastArgs.getUseExternalBrowser());
        RTW_TraceArgs.instance.setModel2CodeSrc(lastArgs.getModel2CodeSrc());
    }    

    // get highlight url using sid
    if (RTW_TraceArgs.instance.hasSid()) {
        aHash = getCodeLines();  
    }
    // parse hash to look for msg=...&block=... pattern
    RTW_TraceArgs.instance.parseCommand(aHash);
    // parse hash to look for file.c:10,12&file.h:10,12 
    RTW_TraceArgs.instance.parseUrlHash(aHash);

    // hide navigation buttons if not in MATLAB
    if (RTW_TraceArgs.instance.getUseExternalBrowser() && tocDocObj.getElementById) {
        var o = tocDocObj.getElementById("nav_buttons");
        if (o != null) {
            o.style.display = "none";
        }
    }

    // hide web view frameset if model2code_src is model
    if (RTW_TraceArgs.instance.getModel2CodeSrc() === "model") {
        var o = top.document.getElementById('rtw_webviewMidFrame');
        if (o) {
            o.rows = "100%,0%";
        }
    }

    // stop onload when it has been loaded
    if (window.location.search.indexOf("loaded=true") > 0 
        && top.rtwreport_document_frame.location.href !== "about:blank" && forceReload !== true) {
        updateHyperlinks();
        return;
    }  
    
    // modify modelref links
    update_modelref_report_link(top.rtwreport_contents_frame.document);
    try {
        // ignore browser security error 
        update_modelref_report_link(top.rtwreport_document_frame.document);
    } catch(e) {};

    // redirect the page based on the url    
    var initPage = null;
    if (RTW_TraceArgs.instance.getNumFiles()) {
        var fileLinks = RTW_TraceInfo.getFileLinks(tocDocObj);
        RTW_TraceInfo.instance = new RTW_TraceInfo(fileLinks);
        RTW_TraceInfo.instance.removeHighlighting()
        var numFiles = RTW_TraceArgs.instance.getNumFiles();
        var tLines = 0;
        for (var i = 0; i < numFiles; ++i) {
            RTW_TraceInfo.instance.setLines(RTW_TraceArgs.instance.getFile(i),RTW_TraceArgs.instance.getLines(i));
            tLines += RTW_TraceArgs.instance.getLines(i).length;
        }
        RTW_TraceInfo.instance.setTotalLines(tLines);
        if (aPanel == false) {
            RTW_TraceInfo.instance.setDisablePanel(true);
        }
        var initFile = RTW_TraceArgs.instance.getInitFile();
        RTW_TraceInfo.instance.setInitLocation(initFile,RTW_TraceArgs.instance.getInitLine());
        if (!hasInCodeTrace()) {
            initPage = RTW_TraceInfo.instance.getCurrentHRef();
        } else {
            initPage = initFile;
        }
    } else {
        // catch error that document frame is in another domain
        try {
            var fileDocObj = top.rtwreport_document_frame.document;
            if (fileDocObj.location && (!fileDocObj.location.href || fileDocObj.location.href == "about:blank")) {
                var summaryPage = tocDocObj.getElementById("rtwIdSummaryPage");
                var tracePage = tocDocObj.getElementById("rtwIdTraceability");
                if (summaryPage) {
                    initPage = summaryPage.href;
                } else if (tracePage) {
                    initPage = tracePage;
                }
            }
        } catch(e) {};
    }
    if (RTW_TraceArgs.instance && RTW_TraceArgs.instance.fMessage) {
        // display diagnostic message
        var linkId = "rtwIdMsgFileLink";
        var msgFile = tocDocObj.getElementById(linkId);
        if (msgFile && msgFile.style) {
            msgFile.style.display = "block";
            // Highlight the background of msg link
            tocHiliteById(linkId);      
        }
        initPage = "rtwmsg.html";
    }
    if (initPage) {
        var is_same_page = false;
        try {
            var fileDocObj = top.rtwreport_document_frame.document;
            is_same_page = isSamePage(fileDocObj.location.href, initPage);
        } catch(e) {};     
        if (document.getElementById("rtwreport_document_frame")) {
            document.getElementById("rtwreport_document_frame").setAttribute("src", initPage);
        } else {
            top.rtwreport_document_frame.location.href = initPage;
        }
                
        if (is_same_page) {
            // Goto the same page won't trigger onload function.
            // Call it manuelly to highligh new code location.
            rtwFileOnLoad(top.rtwreport_document_frame.document);        
        } 
    }
}

// Compare if href1(i.e. file:///path/file1.html#222) and href2(i.e.file2.html) are same pages.
// isSamePage return true if file1 == file2.
function isSamePage(href1, href2) {
    var page1 = href1.substring(href1.lastIndexOf('/')+1,href1.lastIndexOf('.html'));
    var page2 = href2.substring(href2.lastIndexOf('/')+1,href2.lastIndexOf('.html'));
    return (page1 == page2);
}

// Callback for main document loading
function rtwMainOnLoad() {    
    rtwMainOnLoadFcn(document,null,true, false);
    var newUrl;
    // modify history state to avoid reload from pressing back 
    if (RTW_TraceArgs.instance && !RTW_TraceArgs.instance.getUseExternalBrowser() && 
        typeof window.history.replaceState === "function") {
        if (window.location.search.length > 0) {
            if (window.location.search.indexOf("loaded=true") === -1) {
                newUrl = document.location.pathname + window.location.search + '&loaded=true';
            } else {
                newUrl = document.location.pathname + window.location.search;
            }
        } else {
            newUrl = document.location.pathname + window.location.search + '?loaded=true';
        }
        window.history.replaceState("","",newUrl);
    }
}

// Helper function for traceability report
function rtwMainReload(location) {
    // remove highlight filename and lines before reloading the page
    if (RTW_TraceInfo.instance)
        RTW_TraceInfo.instance.removeHighlighting();  
    rtwMainOnLoadFcn(document,location,true,true);
}

function rtwMainReloadNoPanel(location) {
    rtwMainOnLoadFcn(document,location,false,true);
}

// Callback for hyperlink "Remove Highlighting"
function rtwRemoveHighlighting() {
    if (RTW_TraceInfo.instance)
        RTW_TraceInfo.instance.removeHighlighting();
    if (rtwSrcFrame()) {
        rtwSrcFrame().focus();
    }
    if (hasInCodeTrace()) {
        removeInCodeTraceHighlight();
    }
}

// Display diagnostic message in document frame
function rtwDisplayMessage() {
    var docObj = top.rtwreport_document_frame.document;
    var msg = docObj.getElementById(RTW_TraceArgs.instance.fMessage);
    if (!msg) {
        msg = docObj.getElementById("rtwMsg_notTraceable");
    }
    if (msg && msg.style) {
        msg.style.display = "block"; // make message visible
        var msgstr = msg.innerHTML;
        // replace '%s' in message with block name
        if (top.RTW_TraceArgs.instance) {
            var sid = top.RTW_TraceArgs.instance.getBlock();
            if (sid) {
                var block = sid;
                if (top.RTW_rtwnameSIDMap && top.RTW_rtwnameSIDMap.instance && top.RTW_rtwnameSIDMap.instance.getRtwname(sid)) {
                    block = top.RTW_rtwnameSIDMap.instance.getRtwname(sid).rtwname;
                    block = block.replace("<", "&lt;").replace(">", "&gt;");
                } else {
                    block = sid;
                }
                if (block) {
                    msgstr = msgstr.replace("%s", block);
                }
            }
        }
        msg.innerHTML = msgstr;
    }
}

function updateHyperlinks() {
    docObj = top.rtwreport_document_frame;
    if (docObj && docObj.document) {
        if (RTW_TraceArgs.instance === null || !RTW_TraceArgs.instance.getUseExternalBrowser()) {
            var plain_link =  docObj.document.getElementById("linkToText_plain");
            if (plain_link && plain_link.href && plain_link.href.indexOf("matlab:coder.internal.editUrlTextFile") === -1 ) {
                plain_link.href = "matlab:coder.internal.editUrlTextFile('" + str2StrVar(plain_link.href) + "')";
            }          
            var alink = docObj.document.getElementById("linkToCS");
            var linkCmd = "matlab:coder.internal.viewCodeConfigsetFromReport";
            if (alink && alink.href && alink.href.indexOf(linkCmd) === -1) {
                alink.href = linkCmd+ "('" + str2StrVar(alink.href) + "');";
                if(alink.style) {
                    alink.style.display = "";
                    hidden_link = docObj.document.getElementById("linkToCS_disabled");
                    if (hidden_link) {
                        hidden_link.style.display = "none";
                    }
                }
            }
        } else {
            var alink = docObj.document.getElementById("linkToCS");
            if (alink && alink.style) {
                alink.style.display = "none";
                hidden_link = docObj.document.getElementById("linkToCS_disabled");
                if (hidden_link)
                    hidden_link.style.display = "";
            }
            if (typeof docObj.document.getElementsByClassName === "function") {
                alinks = docObj.document.getElementsByClassName("callMATLAB");
            } else if (typeof docObj.document.getElementsByName === "function") {
                alinks = docObj.document.getElementsByName("callMATLAB");
            } else {
                alinks = [];
            }
            alink = docObj.document.getElementById("CodeGenAdvCheck");
            if (alink && alink.href && alink.href.indexOf("externalweb=true")===-1) {
                alink.href = alink.href + "?externalweb=true";
            }

            if (typeof docObj.document.getElementsByName === "function") 
                var objs = docObj.document.getElementsByName("MATLAB_link");
            else 
                objs = [];
            for (var objIndex = 0; objIndex < objs.length; ++objIndex) {     
                objs[objIndex].style.display = "none";
            }
        }
    }
    updateCode2ModelLinks(docObj.document);
    // modify modelref links
    update_modelref_report_link(top.rtwreport_contents_frame.document);
    try {
        // ignore browser security error 
        update_modelref_report_link(top.rtwreport_document_frame.document);
    } catch(e) {};
}

function update_modelref_report_link(docObj) {
    if (docObj.getElementsByName) {
        var arg = "";
        if (RTW_TraceArgs.instance && !RTW_TraceArgs.instance.getUseExternalBrowser()) {
            arg = "?useExternalBrowser=false";
        }
        if (RTW_TraceArgs && RTW_TraceArgs.instance && RTW_TraceArgs.instance.getModel2CodeSrc() != null) {
            if (arg.length > 0)
                arg = arg + "&model2code_src=" + RTW_TraceArgs.instance.getModel2CodeSrc();
            else
                arg = "?model2code_src=" + RTW_TraceArgs.instance.getModel2CodeSrc();
        }
        if (arg.length > 0) {
            links = docObj.getElementsByName('external_link');
            for (var link_idx = 0; link_idx < links.length; ++link_idx) {
                links[link_idx].href = links[link_idx].href + arg;
            }
        }
    }
}

function rtwResizeFrame(f) {
    if (f) {
        f.style.height = f.contentWindow.document.body.scrollHeight + "px";
    }
}

function rtwPageOnLoad(id) {
    // highlight toc entry
    tocHiliteById(id);
    // restore elements state
    if (top && top.restoreState) {
        if (top.rtwreport_contents_frame && top.rtwreport_contents_frame.document)
            top.restoreState(top.rtwreport_contents_frame.document);
        if (top.rtwreport_document_frame && top.rtwreport_document_frame.document) {
            top.restoreState(top.rtwreport_document_frame.document);
            rtwResizeFrame(top.rtwreport_document_frame.document.getElementById("rtwIdContentsIframe"));
        }
    }
    updateHyperlinks();
}

// highlight code after changeSys
function rtwChangeSysCallback(sid) {
    if (sid == "" || typeof RTW_Sid2UrlHash == "undefined" || !RTW_Sid2UrlHash.instance)
        return false;
    urlHash = RTW_Sid2UrlHash.instance.getUrlHash(sid);
    if (urlHash != undefined) {
        if (RTW_TraceArgs && RTW_TraceArgs.instance && 
            !RTW_TraceArgs.instance.getUseExternalBrowser())
            urlHash = (urlHash == "")? "?useExternalBrowser=false" : 
            urlHash+"&useExternalBrowser=false";
        rtwMainReload(urlHash, true);
        return true;
    } else {
        // remove highlighting from traceinfo
        rtwRemoveHighlighting();
        return false;
    }
}

function emlFileOnload(docObj) {
    var loc = docObj.location;
    if (loc.hash) {
        var line = loc.hash.substring(1);
        hiliteEmlLine(docObj, line);                        
    }   
}

function hiliteEmlLine(docObj, line) {
    var bgColor;
    if (top.HiliteCodeStatus)
        bgColor = "#66CCFF";
    else
        bgColor = "#E8D152";
    // unhighlight
    if (typeof docObj.HiliteLine != "undefined") {
        trObj = docObj.getElementById("LN_"+docObj.HiliteLine);
        if (trObj != null) {
            trObj.style.backgroundColor = "";                   
        }
    }   
    // hilighlight
    trObj = docObj.getElementById("LN_"+line);
    if (trObj != null) {
        trObj.style.backgroundColor = bgColor;
        docObj.HiliteLine = line;
    }
}

function emlLineOnClick(docObj,sid,line) {
    if (top) {
        top.HiliteCodeStatus = top.rtwChangeSysCallback(sid);        
    }
    hiliteEmlLine(docObj, line);
}

function updateCode2ModelLinks(docObj) {
    var webviewFrame = top.document.getElementById('rtw_webviewMidFrame');
    var link2model = false;
    var isTestHarness = false;
    if (top.testHarnessInfo && top.testHarnessInfo.IsTestHarness === "1") {
        isTestHarness = true;
    }
    if (webviewFrame || isTestHarness) {
        if (webviewFrame && RTW_TraceArgs.instance && 
            (RTW_TraceArgs.instance.getModel2CodeSrc() !== "model" ||
             RTW_TraceArgs.instance.getUseExternalBrowser())
           ) {
            hiliteCmd = "javascript:top.rtwHilite(";
        } else {
            hiliteCmd = "matlab:coder.internal.code2model(";
            link2model = true;
        }
        var objs = docObj.getElementsByName('code2model');
        var o = null;
        var str = '';
        var sid = '';
        var pattern = "'code2model',";
        for (var objIndex = 0; objIndex < objs.length; ++objIndex) {     
            o = objs[objIndex];
            str = o.href.substring(o.href.indexOf('(')+1);
            if (str.indexOf(pattern) > -1) {
                str = str.substring(str.indexOf(pattern) + pattern.length);
            }
            o.href = hiliteCmd + str;
            if (link2model && isTestHarness) {
                sid = str.substring(0, str.indexOf(")"));
                o.href = hiliteCmd + sid + ",'" +
                    top.testHarnessInfo.HarnessName+ "','" + 
                    top.testHarnessInfo.HarnessOwner+ "','" + 
                    top.testHarnessInfo.OwnerFileName + "');";
            }
        }
    }
}

function rtwHilite(aBlock,aParentSID) {
    if (aBlock.indexOf('-') !== -1) { 
        // remove sid range: model:sid:2-10 => model:sid 
        var s; 
        s = aBlock.split(':'); 
        if (s.length > 0) { 
            s = s[s.length-1]; 
            if (s.indexOf('-') != -1) { 
                aBlock = aBlock.substring(0, aBlock.lastIndexOf(':')); 
            } 
        } 
    } 
    if (typeof aParentSID === "undefined") {
        if (top.RTW_SidParentMap && top.RTW_SidParentMap.instance)
            aParentSID = top.RTW_SidParentMap.instance.getParentSid(aBlock);
        else
            aParentSID = aBlock;
    }
    top.HiliteCodeStatus = true;
    
    // webview 2 defines an interface api, call slwebview.
    if (top.slwebview) {
        // webview 2.x
        if (top.codeToWebView(aBlock, aParentSID) === -1) {
            alert("Cannot highlight block in model Web view. It may not be exported.");
        }
    
    } else {
        // webview 1.x
        if (hiliteBlockForRTWReport(aBlock,aParentSID) === false) {
            if (hiliteBlockForRTWReport(aBlock, aBlock) === false) {
                rtwHilite(aParentSID);
            }
        }
    }
}

function str2StrVar(str) {
    return str.replace(/'/g,"''");
}
window.onload=rtwMainOnLoad;

// handle incode traceability highlighting
function inCodeTraceOnload() {  
    var tocDocObj = top.rtwreport_contents_frame.document;
    if (!top.RTW_TraceArgs.instance) {
        var summaryPage = tocDocObj.getElementById("rtwIdSummaryPage");                 
        top.rtwreport_document_frame.location.href = summaryPage.href;  
        return;
    }

    var files = top.RTW_TraceArgs.instance.getFile();
    if (files.length === 0) {
        if (top.RTW_TraceArgs.instance) {
            var block = top.RTW_TraceArgs.instance.getBlock();
            block = block.replace("<", "&lt;").replace(">", "&gt;");
        }
        top.rtwreport_document_frame.document.write("<pre>No traceability information for block " + block + ".</pre>");
        return;
    };

    var fileLinks = RTW_TraceInfo.getFileLinks(tocDocObj);
    RTW_TraceInfo.instance = new RTW_TraceInfo(fileLinks);

    // update filelist with num of highlighted lines
    var tocDoc = top.rtwreport_contents_frame.document;
    var tLines = 0;
    for (var i=0; i<files.length;i++) {
        var fileIdx = top.RTW_TraceArgs.instance.getFileIdx(files[i]);
        if (typeof fileIdx !== "undefined") {
            var rows = top.RTW_TraceArgs.instance.getRows(fileIdx);
            var linkNode =  tocDoc.getElementById(files[i]);
            updateNumHighlightedLines(linkNode, rows.length);
            RTW_TraceInfo.instance.setLines(files[i], rows);
            tLines += rows.length;
        }
    }
    // set number of total lines
    RTW_TraceInfo.instance.setTotalLines(tLines);
    // update highligthed from
    if (RTW_TraceArgs.instance.getNumBlocks() === 1) {
        var node = tocDoc.getElementById("rtwIdTraceBlock");
        if (node) node.textContent = RTW_TraceArgs.instance.getBlock();
    }
    // set the initial file and line
    fileIdx = top.RTW_TraceArgs.instance.getFileIdx(files[0]);
    rows = top.RTW_TraceArgs.instance.getRows(fileIdx);
    RTW_TraceInfo.instance.setInitLocation(files[0],rows[0]);

    // highlight first file
    top.rtwreport_document_frame.location.href = files[0];
    return;
}

function updateNumHighlightedLines(linkObj, aNumLines) {
    var parent = linkObj.parentNode;
    if (parent && parent.childNodes && parent.childNodes.length > 1) {
        var spanNodes = parent.getElementsByTagName('span');
        var len = spanNodes.length;
        if (len > 0) {
            if (aNumLines > 0) {
                /* display number of matches */
                spanNodes.item(len-1).innerHTML = "&nbsp;("+aNumLines+")";
            } else {
                /* clear number of matches */
                spanNodes.item(len-1).innerHTML = "";
            }
        }
    }
}

function setupInStudio() {
    if (top.whole) {
        var tmp = top.whole.rows.split(",");    
        tmp[0] = "35px";
        top.whole.rows = tmp.join();    
    }
    if (top.main) {
        var tmp = top.main.cols.split(",");
        tmp[0] = "0";
        top.main.cols = tmp.join();
    }
    // add file list to source file
    if (top.Html2SrcLink && top.Html2SrcLink.instance && top.fileSelector) {
        var myDoc = top.fileSelector.document;
        var fileSelector = myDoc.createElement("select");
        fileSelector.id = "fileSelector";
        fileSelector.onchange = top.fileSelectorOnChange;
        var filename;
        var filelink;
        fileSelector.innerHTML += "<option value='" + 
            top.rtwreport_contents_frame.document.getElementById('rtwIdSummaryPage').href +
            "'>Summary</option>"; 
        for (var i=0; i < top.fileList.length; i++) {
            filename = top.fileList[i];
            filelink = top.Html2SrcLink.instance.getLink2Src(filename);
            fileSelector.innerHTML += "<option value='" + filename + "'>" + filelink.substring(filelink.lastIndexOf('/')+1); + "</option>";
        }
        var bodyNode = myDoc.body;
        bodyNode.insertBefore(fileSelector, bodyNode.firstElementChild);
        var textNode = myDoc.createElement("span");
        textNode.innerHTML = "Goto: ";
        bodyNode.insertBefore(textNode, fileSelector);
        var myCss = myDoc.createElement("link");
        myCss.type = "text/css";
        myCss.rel = "stylesheet";
        myCss.href = "rtwreport.css";
        myDoc.getElementsByTagName("head")[0].appendChild(myCss);
    }
}

function toggleNavSideBar(val) {
    if (top.main) {
        var tmp = top.main.cols.split(",");    

        if (val === "on") {
            tmp[tmp.length-1] = "15px";        
        } else {
            tmp[tmp.length-1] = "0";        
        }
        top.main.cols = tmp.join();    
        if (top.rtwreport_nav_frame) 
            top.rtwreport_nav_frame.location.href = "nav.html";    
    }
};

function toggleNavToolBar(val) 
{    
    var midFrame = rtwMidFrame();
    if (midFrame) {
        var tmp1 = midFrame.rows.split(",");
        var frameIdx = getNavToolbarFrameIdx();
        if (val === "on") {
            tmp1[frameIdx] = "40px";
        } else {
            tmp1[frameIdx] = "0";
        }    
        midFrame.rows = tmp1.join();
        if (top.rtwreport_navToolbar_frame) {
            top.rtwreport_navToolbar_frame.location.href = "navToolbar.html";
        }
    }
};

var GlobalConfig = {
    navHiliteColor: "#0000ff",
    fileLinkHiliteColor: "#ffff99",
    navToolbarBgcolor: "ivory",
    offset: 10,
    hiliteToken: false
};
var NavSideBarState = {
    calLineHeight: 0, 
    overLink: false,
    linkTarget: null,
    lastLinkTarget: null,
    linkTargetIdx: 0
}
function drawNavSideBar() {
    var rectHeight = 1;
    if (!top || !top.rtwreport_document_frame || !top.rtwreport_nav_frame) return;
    
    if (!top.RTW_TraceArgs.instance) return;
    var fileIdx = top.RTW_TraceArgs.instance.getFileIdx();
    if (fileIdx === undefined) return;
    var rows = top.RTW_TraceArgs.instance.getRows(fileIdx);                
    if (rows.length === 0) return; // no highlighted line 
    
    var codeTbl = top.rtwreport_document_frame.document.getElementById("codeTbl");
    if (!codeTbl) return; // no code table
    
    var nRows = codeTbl.rows.length + 1;
    var canvas = top.rtwreport_nav_frame.document.getElementById("canvas");                
    canvas.width = top.rtwreport_nav_frame.innerWidth;
    canvas.height = top.rtwreport_nav_frame.innerHeight-2;
    NavSideBarState.calLineHeight = canvas.height/nRows;
    if (canvas.getContext) {
        var ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        // fill background 
        ctx.fillStyle = GlobalConfig.navToolbarBgcolor;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = GlobalConfig.navHiliteColor;
        for (var i=0;i<rows.length;i++) {
            ctx.fillRect(0, Number(rows[i])*NavSideBarState.calLineHeight, canvas.width, rectHeight);
        }
        if (canvas.addEventListener) { 
            canvas.addEventListener("mousemove", navBarOnMousemove, false);
            canvas.addEventListener("click", navBarOnClick, false);
        } else if (canvas.attachEvent) {
            canvas.attachEvent("mousemove", navBarOnMousemove);
            canvas.attachEvent("click", navBarOnClick);
        }
    }
}

function navBarOnMousemove(e) {
    var y = e.clientY;
    var tolerable_range = 5;
    if (!top.RTW_TraceArgs.instance || !top.rtwreport_nav_frame) return;
    var fileIdx = top.RTW_TraceArgs.instance.getFileIdx();
    var rows = top.RTW_TraceArgs.instance.getRows(fileIdx);
    var lineLoc,nextLineLoc;
    top.rtwreport_nav_frame.document.body.style.cursor="";
    NavSideBarState.overLink = false;
    NavSideBarState.linkTarget = null;                 
    NavSideBarState.linkTargetIdx = null;
    for (var i=0;i<rows.length;i++) {
        loc = rows[i]*NavSideBarState.calLineHeight;
        // if within the tolerable range
        if (Math.abs(y-loc) <= tolerable_range) {
            top.rtwreport_nav_frame.document.body.style.cursor="pointer";
            var canvas = top.rtwreport_nav_frame.document.getElementById("canvas");                
            canvas.title = "navigate to line " + rows[i];
            NavSideBarState.overLink = true;
            NavSideBarState.linkTarget = rows[i];
            NavSideBarState.linkTargetIdx = i;
            break;
        } 
    }
}

function navBarOnClick(e) {
    if (NavSideBarState.overLink && top.rtwreport_document_frame) {
        rmTagToCurrentLine(); // remove current line tag
        top.RTW_TraceInfo.instance.setCurrLineIdx(NavSideBarState.linkTargetIdx);
        top.rtwreport_document_frame.document.location.href=RTW_TraceInfo.instance.getCurrentHRef();
        top.addTagToCurrentLine(); // add current line tag
        RTW_TraceInfo.instance.updateNavState();
    }                
}

function removeInCodeTraceHighlight() {
    var docObj = top.rtwreport_document_frame.document;
    toggleNavSideBar("off");
    toggleNavToolBar("off");
    var nodes = docObj.getElementsByClassName("hilite");
    // nodes is a live nodeList. Changing className modifies the list.
    while(nodes.length) {
        nodes[0].className = nodes[0].className.replace("hilite", "");
    }
    var nodes = docObj.getElementsByClassName("hiliteCurrentLine");
    // nodes is a live nodeList. Changing className modifies the list.
    while(nodes.length) {
        nodes[0].className = nodes[0].className.replace("hiliteCurrentLine", "");
    }
    // reset RTW_TraceArgs.instance
    RTW_TraceArgs.instance = null;
    // remove highlight in content panel except the filelink
    if (RTW_TraceInfo && RTW_TraceInfo.instance) {
        var currFileIdx = RTW_TraceInfo.instance.getCurrFileIdx();
        RTW_TraceInfo.instance.removeHighlighting();
        RTW_TraceInfo.instance.highlightFileLink(currFileIdx);
    }
}

function getInspectWindow() {
    var divObj = document.createElement("div");
    divObj.id = "popup_window";
    return divObj;    
}
function getInspectData(file, anchorObj) {   
    var metricsData = null;
    var propObj = null;
    var type = null;
    var size = null;
    var cm;
    var srcFileName = RTW_TraceInfo.toSrcFileName(file);
    if (top.rtwreport_nav_frame && top.rtwreport_nav_frame.CodeMetrics && 
        top.rtwreport_nav_frame.CodeMetrics.instance && 
        top.RTW_TraceArgs && top.RTW_TraceArgs.instance && 
        !top.RTW_TraceArgs.instance.getUseExternalBrowser()) {
        cm = top.rtwreport_nav_frame.CodeMetrics.instance;
    }
    if (cm && cm.getMetrics) {
        metricsData = cm.getMetrics(anchorObj.text);
        if (!metricsData) {
            // try static token
            metricsData =  cm.getMetrics(srcFileName + ":" + anchorObj.text);
        }
        if (metricsData) {            
            type = metricsData.type;
            if (type === "var") {
                type = "Global Variable";
                size = "(" + metricsData.size + " byte)";
            } else if (type === "fcn") {
                type = "Function";
                if (metricsData.stackTotal === -1) {
                    size = "(stack: " + metricsData.stack + " byte, total stack: recursion)";
                } else {
                    size = "(stack: " + metricsData.stack + " byte, total stack: "
                        + metricsData.stackTotal + " byte)";
                }
            }            
        }
    }    
    if (type === null) {
        var defObj;
        if (top.CodeDefine.instance.def[srcFileName + ":" + anchorObj.text]) {
            defObj = top.CodeDefine.instance.def[srcFileName + ":" + anchorObj.text];
        } else if (top.CodeDefine.instance.def[anchorObj.text]) {
            defObj = top.CodeDefine.instance.def[anchorObj.text];
        }
        if (defObj) {
            type = defObj.type;
            if (type === "var") {
                type = "Variable";
            } else if (type === "fcn") {
                type = "Function";
            } else if (type === "type") {
                type = "Type";
            }
            size = "";
        }
    }   
    var propObj = document.createElement("div");
    propObj.id = "token_property";
    
    var ulObj = document.createElement("ul");
    ulObj.className = "popup_attrib_list";
    if (type === null) {
        ulObj.innerHTML = "Navigate to model";
    } else {
        ulObj.innerHTML = "<li>" + type + ": <var>" + anchorObj.text + "</var></li><li>"+
            size + "</li>";
    }
    propObj.appendChild(ulObj);   
    
    return propObj;
}
function getInspectLink(file, pathname, anchorObj) {
    var model = top.reportModel;
    var tokenId = anchorObj.id;
    var navObj = document.createElement("div");
    navObj.id = "token_usage_nav";
    ulObj = document.createElement("ul");
    ulObj.id = "token_nav_links";
    ulObj.className="popup_attrib_list";
    var srcFileName = RTW_TraceInfo.toSrcFileName(file);
    var defObj;
    if (top.CodeDefine.instance.def[srcFileName + ":" + anchorObj.text]) {
        defObj = top.CodeDefine.instance.def[srcFileName + ":" + anchorObj.text];
    } else if (top.CodeDefine.instance.def[anchorObj.text]) {
        defObj = top.CodeDefine.instance.def[anchorObj.text];
    }
    var line = anchorObj.id.substring(0,anchorObj.id.indexOf("c"));
    // link to model
    if (top.TraceInfoFlag && top.TraceInfoFlag.instance && 
        top.TraceInfoFlag.instance.traceFlag[srcFileName+":"+anchorObj.id]) {
        return null;
    }
    // link to def/decl
    if (defObj) {
        var filename = defObj.file.split(/\//);
        filename = filename[filename.length-1];
        ulObj.innerHTML += "<li><i>" + anchorObj.text + "</i> defined at <a target='rtwreport_document_frame' onclick=\"top.tokenLinkOnClick(event)\" href='" + defObj.file + "#" + defObj.line +
            "'>" + RTW_TraceInfo.toSrcFileName(filename) + " line " + defObj.line + "</a></li>";
    }
    navObj.appendChild(ulObj);
    return navObj;
}

var LastHiliteTokenId = null;
function rmHiliteClickedToken() {
    if (LastHiliteTokenId) {
        var o = top.rtwreport_document_frame.document.getElementById(LastHiliteTokenId);
        if (o) {
            o.className = o.className.replace("hiliteToken", "");
        }
    }
}
function hiliteClickedToken(elem) {
    rmHiliteClickedToken();
    LastHiliteTokenId = elem.id;
    elem.className += " hiliteToken";
}

var initLine = null;
function scrollToInitLine() {
    if (initLine) {
        var lineElem = top.rtwreport_document_frame.document.getElementById(initLine);
        if (lineElem) {
            lineElem.scrollIntoView();
        }
    }
}

function scrollToLineBasedOnHash(hashValue) {
    // move to the current highlight line if the hash is not empty
    if (hashValue === "") {
        if (top.RTW_TraceInfo.instance && top.RTW_TraceInfo.instance.getCurrLine() !== null) {
            top.rtwreport_document_frame.document.location.href=top.RTW_TraceInfo.instance.getCurrentHRef();
            top.initLine = top.rtwreport_document_frame.document.location.hash.substr(1);
        }
    } else {
        // scroll and hilite line
        hashValue = hashValue.substr(1);
        if (isNaN(hashValue)) {
            // #fcn_name
            var pattern = "+newPage";
            if (hashValue.indexOf(pattern) != -1) {
                hashValue = hashValue.replace(pattern, '');
                var lineElem = top.rtwreport_document_frame.document.getElementById(hashValue);
                initLine = hashValue; // save initLine in case the dom is updated later by anootation
                if (lineElem) {
                    lineElem.scrollIntoView(); 
                    addTagToCurrentLine();
                }

            } else {
                var token = null;
                pattern = ["var_", "fcn_", "type_"];
                for (var i =0; i < pattern.length; i++) {
                    if (hashValue.indexOf(pattern[i]) === 0) {
                        token = hashValue.substr(pattern[i].length);
                        break;
                    }
                }
                if (token !== null && top.CodeDefine && top.CodeDefine.instance) {
                    var addr;
                    var filename = location.pathname.split(/\//);
                    filename = filename[filename.length-1];  
                    var srcFileName;
                    if (top.RTW_TraceInfo) {
                        srcFileName = top.RTW_TraceInfo.toSrcFileName(filename);
                    }
                    if (top.CodeDefine.instance.def[srcFileName + ":" + token]) {
                        addr = top.CodeDefine.instance.def[srcFileName + ":" + token];
                    } else {
                        addr = top.CodeDefine.instance.def[token];
                    }
                    if (addr) {
                        hilite_line(addr.line);
                    }
                } else { // token id like #line"c"#col
                    if (hashValue.indexOf("c") !== -1) {
                        hilite_line(hashValue.substr(0, hashValue.indexOf("c")), hashValue);
                    }
                }
            }
        } else { // #line
            hilite_line(hashValue);
        }
    }
    return false;
    // hilite line number and scroll with an offset
    function hilite_line(line, tokenId) {
        if (isNaN(line)) return;
        if (!tokenId) {
            tokenId = line;
        }
        var elem = top.rtwreport_document_frame.document.getElementById(tokenId);               
        hiliteClickedToken(elem);
        initLine = offset_line(line);
        scrollToInitLine();
    }
}

function tokenLinkOnClick(event) {
    var alink = event.currentTarget;
    if (alink.pathname === top.rtwreport_document_frame.location.pathname) {
        event.preventDefault();
        scrollToLineBasedOnHash(alink.hash);
    }
    return false;
}
function inspectToken(file, pathname, event) {
    var height = "70px";
    // show inspect data
    if (top.rtwreport_inspect_frame) { 
        var windowObj = getInspectWindow();
        var propObj = getInspectData(file, event.currentTarget);
        var navObj = getInspectLink(file, pathname, event.currentTarget);
        if (navObj === null) {
            closeInspectWindow();
            return false;
        }
        if (propObj === null) {
            height = "50px";
        } else {
            windowObj.appendChild(propObj);
        }
        windowObj.appendChild(navObj);
        var data = top.rtwreport_inspect_frame.document.getElementById("popup_window");
        if (data) {
            data.parentNode.replaceChild(windowObj.cloneNode(true), data);
        }
    }
    var offsetHeight = 0;
    var docHeight = 0;
    if (typeof(top.rtwInspectFrame().document.body.offsetHeight) === "number") {
        offsetHeight = top.rtwInspectFrame().document.body.offsetHeight;        
    }
    if (typeof(top.rtwInspectFrame().document.height) === "number") {
        docHeight = top.rtwInspectFrame().document.height;
    }   
    if (offsetHeight > 0) {
        height = ""+offsetHeight+"px";
    } else if (docHeight > 0) {
        height = ""+docHeight+"px";
    }   
    setInspectWindow(height);
    return false;
}
function setInspectWindow(height) {
    // show inspect code frame
    var midFrame = rtwMidFrame();
    if (midFrame) {
        var tmp = midFrame.rows.split(",");
        tmp[getInspectFrameIdx()] = height;    
        midFrame.rows = tmp.join();
    }
}
function closeInspectWindow() {
    setInspectWindow(0);
    return false;
}

// set the trace number in the navigation toolbar
function setTraceNumber() {
    if (RTW_TraceInfo.instance) {
        var aFrame = rtwNavToolbarFrame();
        if (aFrame) {
            var node = aFrame.document.getElementById("rtwIdTraceNumber");
            // calculate current line index over total highlighted lines
            var currNum = RTW_TraceInfo.instance.getCurrLineIdx();
            for (var idx=0;idx<RTW_TraceInfo.instance.getCurrFileIdx();idx++) {
                currNum += RTW_TraceInfo.instance.getNumLines(idx);
            }
            if (node) {
                node.innerHTML = String(currNum+1) + " of " + String(RTW_TraceInfo.instance.getTotalLines());
            }
        }
    }
}

function offset_line(line, offset) {
    if (offset == undefined)
        offset = GlobalConfig.offset;
    if (offset > 0)
        line = (line > GlobalConfig.offset ? line - GlobalConfig.offset : 1);
    return line;
}

function load_js(frame, file) {
    var h = frame.document.getElementsByTagName("head")[0];
    var o = h.getElementsByTagName('script');
    for (var i=0;i<o.length;++i) {
        if (o[i].getAttribute("src") === file) {
            h.removeChild(o[i]);
        }
    }
    var s = top.document.createElement("script");
    s.type = "text/javascript";
    s.src = file;
    h.appendChild(s);
}

function reqOnClick(event) {
    top.hiliteClickedToken(event.currentTarget);
    return true;
}
function resize_NavToolbar_frame() {
    resize_frame(getNavToolbarFrameIdx(), rtwNavToolbarFrame().document.height);
}
function resize_frame(id, height) {
    if (height) {
        var midFrame = top.rtwMidFrame();
        var tmp = midFrame.rows.split(",");
        if (tmp[id] !== "0%" && tmp[id] !== "0") {
            tmp[id] = "" + height - 8 + "px";
            midFrame.rows = tmp.join();
        }
    }
}
function getNavToolbarFrameIdx() {
    return 0;
}
function getInspectFrameIdx() {
    return 2;
}
function load_metrics() {
    var alink = top.document.createElement("a");
    alink.href = "metrics.js";
    if (top.RTW_TraceArgs && top.RTW_TraceArgs.instance && !top.RTW_TraceArgs.instance.getUseExternalBrowser()) {
        try {
            load_js(top.rtwreport_nav_frame, alink.href);
        } catch (err) {};
    }
}

function getNavFrame() {
    if (hasWebviewFrame()) {
        return rtwTocFrame();
    } else {
        return rtwNavToolbarFrame();
    }
}

function hasWebviewFrame() {
    if (top.document.getElementById('rtw_webviewMidFrame')) {
        return true;
    } else {
        return  false;
    }
}
function hasInCodeTrace() {
    return (typeof(Html2SrcLink) === "function") && !hasWebviewFrame();
}
function uniqueRows(rows) {
    return rows.sort(function(a,b) {return a-b}).filter(
        function(el,idx, arr) {
            if (idx===arr.indexOf(el)) return true; return false;
        }
    );
}
function fileSelectorOnChange(event) {
    var o = top.Html2SrcLink.instance.getLinkFromRoot(event.currentTarget.value)
    if (o) {
        top.rtwreport_document_frame.location.href = o;
    } else {
        top.rtwreport_document_frame.location.href = event.currentTarget.value;
    }
}
function getBuildDir() {
    var relPathToBuildDir = top.relPathToBuildDir.substr(0,top.relPathToBuildDir.lastIndexOf('/')+1);
    var a = document.createElement('a');
    a.href = relPathToBuildDir;
    var ret = decodeURI(a.pathname);
    if (top.isPC && ret[0] === "/") {
        ret = ret.substr(1);
    }
    ret = ret.replace(new RegExp("/", 'g'), top.fileSep);
    return ret;
}

function getCodeLocation() {
    var codeLocation = getBuildDir();
    // make build dir link in report visible by clearing 'display' style
    docObj = top.rtwreport_document_frame;
    var alinkTitle = docObj.document.getElementById("sourceLocationTitle");
    if (alinkTitle && alinkTitle.style) {        
        alinkTitle.style.display = "";
    }    
    var alink = docObj.document.getElementById("sourceLocation");
    if (alink && alink.style) {        
        alink.style.display = "";
    }
    return codeLocation;
}
// get code lines for the input SIDs
function getCodeLines()
{
        var codeLocs = "";
        var sid = RTW_TraceArgs.instance.getSID();
        sid = sid.split(",");
        if(sid.length == 1) {
            codeLocs = RTW_Sid2UrlHash.instance.getUrlHash(sid[0]);     
        }
        else {
            var fileLocs = [];
            for(var i=0; i < sid.length; ++i) {
                var locstr = RTW_Sid2UrlHash.instance.getUrlHash(sid[i]);  
                var locs = locstr.split("&");
                for(var j=0; j< locs.length; ++j) {
                    locElems = locs[j].split(":");
                    if(fileLocs[locElems[0]] == null) {
                        fileLocs[locElems[0]] = locElems[1];
                    } 
                    else {
                        fileLocs[locElems[0]] = fileLocs[locElems[0]].concat(",", locElems[1]);                        
                    }
                }
            }
    
            // join all locations
            Object.keys(fileLocs).forEach(function(key) {
                if(codeLocs.length != 0) {
                    codeLocs = codeLocs.concat("&", key, ":", fileLocs[key]);
                } else {
                    codeLocs = codeLocs.concat(key, ":", fileLocs[key]);
                }
            });
        }
    return codeLocs;
}

//add source to frame when _codegen_rpt openend from outisde matlab
function loadDocFrameSource(modelName) {
    const urlParams = new URLSearchParams(window.location.search);
    const opendInExtBrowser = urlParams.get('useExternalBrowser');
    if (opendInExtBrowser === null) {
        document.getElementById('rtwreport_document_frame').src = modelName.concat('_survey.html');
    }
}
