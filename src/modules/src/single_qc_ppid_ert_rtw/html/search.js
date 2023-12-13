// Copyright 2007-2012 The MathWorks, Inc.

function createHighlightSpanStart(num)
{
    return "<span class=\"highlighted\" name=\"highlight" + num + "\">";
}

var str2pos;   // This is a map between a tag stripped string and the original string. 
function getTagStrippedStringAndMap(aString)
{
    var tagStrippedString = new String();

    str2pos = new Array();

    var inTag = false;
    var inScript = false;
    
    for (var strPos = 0; strPos < aString.length; strPos++) {
        if (inTag && aString.charAt(strPos) == '>') {
            inTag = false;
            if (inScript && (strPos > 8) && (aString.slice(strPos, strPos - 8) == '/script>')) {
                inScript = false;
            }            
            continue;
        } else if (!inTag && aString.charAt(strPos) == '<') {
            inTag = true;
            if (!inScript && aString.slice(strPos, strPos + 7) == '<script') {
                inScript = true;
                strPos += 7;
            }
            continue;
        }        
        
        if (inTag == false && inScript == false) {
            tagStrippedString += aString.charAt(strPos);
            str2pos.push(strPos);
        }
    }
    return tagStrippedString;
}

function escapeSpecialChars(string)
{
    // let the browser handle the escaping rather than doing a String.replace
    // call
    var div = document.createElement("div");
    var text = document.createTextNode(string);
    div.appendChild(text);
    var escapedString = div.innerHTML;
    delete div;
    return escapedString;
}

// insert highlight tags into the body. Split it up into multiple sections if necessary
// (i.e. if there is a tag in the middle).
function insertHighlighting(bodyText, previ, i, length, highlightStartTag, highlightEndTag)
{
    var newText = "";    
    newText = bodyText.slice(previ, str2pos[i]);
    
    // insert start
    newText += highlightStartTag;
    
    var str2posprev = str2pos[i];    
    // deal with intermediate tags
    for(var cnt = i; cnt < i+length; cnt++)
    {
        if (str2pos[cnt] > str2posprev+1) // we have jumped some text, so there must be a tag
        {
            // insert end tag
            newText += highlightEndTag;
            
            // insert intermediate body text tags
            newText += bodyText.slice(str2posprev+1, str2pos[cnt]);
            
            // insert start tag
            newText += highlightStartTag;
        }
        newText += bodyText.charAt(str2pos[cnt]);
        str2posprev=str2pos[cnt];
    }
    
    // insert end
    newText += highlightEndTag;
    return newText;
    
}

// check to see if the sequence at position 'i' in taglessString is actually in
// the middle of an escape sequence. We assume escape sequences follow the pattern
// &<sequenceText>;. We check for &nbsp;, &lt;, &gt; and &amp;
function isInEscapedSequence(i, taglessString, searchTerm)
{
    var escapeSeq = /&nbsp;|&lt;|&gt;|&amp;/gi;
    var maxEscapeSeqLength = 6;
    var startPos = 0; 
    var endPos = 0;
    
    // exit if the search term has an escape sequence inside it
    if (escapeSeq.test(searchTerm)) {
        return false;
    }

    // reset the escape sequence
    escapeSeq = /&nbsp;|&lt;|&gt;|&amp;/gi;

    // go back in the string until we find an ampersand or we hit maxEscapeSeqLength characters
    tempI = i;
    var bFound = false;
    while (tempI >= 0 && tempI > (i-maxEscapeSeqLength)) {
        if (taglessString.charAt(tempI) == "&") {
            startPos = tempI;
            bFound = true;
            break;
        } 
        tempI = tempI-1;
        
        // if we hit a ';' in any position other than the first while searching
        // for an ampersand, then we cannot be inside an escape sequence
        if (tempI >= 0 && taglessString.charAt(tempI) == ";") {
            return false;
        }        
    }
    if (!bFound) {
        return false;
    }
    // reset the escape sequence
    escapeSeq = /&nbsp;|&lt;|&gt;|&amp;/gi;

    var subString = taglessString.substring(startPos, startPos + maxEscapeSeqLength);
    return escapeSeq.test(subString);
}

// Adds highlighting to bodyText around searchTerm. Case sensitivity is optional.
// hitCount is used to a) count the number of search matches and b) Generate unique
// name strings for each highlighting SPAN element.
function addHighlight(bodyText, searchTerm, caseSensitive, hitCount) 
{
    var highlightStartTag = ""; 
    var highlightEndTag = "</span>";
  
    var newText = "";
    var i = 0;
    var previ = 0;
    var bodyTextUC = bodyText.toUpperCase();
    
    if (caseSensitive) {
        var taglessString = getTagStrippedStringAndMap(bodyText);
    } else {
        var taglessString = getTagStrippedStringAndMap(bodyTextUC);
    }

    // escape the search term in case the user input '<' or '>' etc
    searchTerm = escapeSpecialChars(searchTerm);

    if (!caseSensitive) {
        var searchTermUC = searchTerm.toUpperCase();
    } 

    // search for subsequent matches
    while (true) {
        if (caseSensitive) {
            i = taglessString.indexOf(searchTerm, i);
        } else {
            i = taglessString.indexOf(searchTermUC, i);
        }
        if (i < 0) break;
        
        // we have a match!
        
        // make sure that the match is not inside an escaped sequence of text
        // such as &nbsp;
        if (isInEscapedSequence(i, taglessString, searchTerm)) {
            i=i+1;
            continue;
        }
        
        // insert highlight tags that cross tag boundaries
        highlightStartTag = createHighlightSpanStart(hitCount);
        hitCount = hitCount+1;
        newText += insertHighlighting(bodyText, previ, i, searchTerm.length, highlightStartTag, highlightEndTag);        
        previ = str2pos[i+searchTerm.length-1]+1;
        
        i = i + searchTerm.length;
    }
    
    newText += bodyText.slice(previ, bodyText.length);
    return [newText, hitCount];
}

function removeHighlight(bodyText) 
{
    // We use regular expressions here rather than a straight text search because 
    // some browsers actually insert double quotes and capitalize.  Also, each highlight
    // name is uniquely numbered in order of discovery
    var highlightStartTag = /<span class=[\"]*highlighted(Current)*[\"]* name=[\"]*highlight[0-9]*[\"]*>/i;
    var highlightEndTag = /<\/span>/i;
    
    var newText = "";

    var startPatternFirstIndex = -1;
    var startPatternLastIndex = -1;
    
    var endPatternFirstIndex = -1;
    var endPatternLastIndex = -1;

    while (highlightStartTag.test(bodyText) === true) {
        startPatternFirstIndex = bodyText.search(highlightStartTag);
        newText += bodyText.substring(0, startPatternFirstIndex);        
        startPatternLastIndex = bodyText.indexOf('>', startPatternFirstIndex+1);
        
        bodyText = bodyText.substr(startPatternLastIndex+1);
        endPatternFirstIndex = bodyText.search(highlightEndTag);
        newText += bodyText.substring(0, endPatternFirstIndex);
        endPatternLastIndex = endPatternFirstIndex+7;
        
        bodyText = bodyText.substr(endPatternLastIndex);
    }
    if (startPatternFirstIndex < 0) {
        return bodyText;
    } else {
        return newText+bodyText;
    }
}

function removeHighlightInAllDocs()
{
    if (top) {
        for (var i = 0; i < top.frames.length; i++) {
            if (top.frames[i].name === "rtwreport_contents_frame" || top.frames[i].name === "rtwreport_document_frame") {
                var currentDoc = top.frames[i].document;
                if (typeof currentDoc.body !== "undefined" && currentDoc.body !== null)
                    currentDoc.body.innerHTML=removeHighlight(currentDoc.body.innerHTML);
            }
        }
    }
}

function findInDoc(searchStringFromSubmitBox, doc, caseSensitive, hitCount) 
{
    var searchBody = doc.body.innerHTML;
    // if the document is empty, or the documents is invalid, return
    if (!doc.body || typeof(searchBody) === "undefined") {
        return [false, hitCount];
    }
        
    // inject highlighting code into html
    var result = addHighlight(searchBody, searchStringFromSubmitBox, caseSensitive, hitCount);
    doc.body.innerHTML = result[0];

    return [true, result[1]];
}

var currentlyHighlightedHit;

function getSpansByName(name)
{
    var allSpans = [];
    for (i = 0; i < top.frames.length; i++) {
        if (top.frames[i].name === "rtwreport_contents_frame" || top.frames[i].name === "rtwreport_document_frame") {
            var currentDoc = top.frames[i].document; 
            var highlightedSpans = currentDoc.getElementsByName(name);
            if (highlightedSpans && highlightedSpans.length && highlightedSpans.length > 0) {
            for (j = 0; j < highlightedSpans.length; j++) {
            allSpans = allSpans.concat(highlightedSpans[j]);
            }
            }
        }
    }
    return allSpans;
}

function isVisibleElement(elementID)
{
    if (elementID)
        return elementID.offsetWidth > 0 || elementID.offsetHeight > 0;
    else 
        return false;
}

function areAllSpansVisible(spans)
{
    isVisible = true;
    for (i = 0; i < highlightedSpans.length; i++) {
        isVisible = isVisible && isVisibleElement(highlightedSpans[i]);
    }
    return isVisible;
}

function getNextVisible()
{
    var isVisible = false;
    var foundVisible = false;
    while (!isVisible) {
        currentlyHighlightedHit = currentlyHighlightedHit + 1;
        highlightedSpans = setCurrentSearchMatchIfVisible(currentlyHighlightedHit);
        if (highlightedSpans && highlightedSpans.length > 0) {
            isVisible = true;
        } else if (currentlyHighlightedHit < totalHits) {
            continue;
        } else {
            // we have reached the end
            isVisible = false;
            currentlyHighlightedHit = 0;
            highlightedSpans = null;
            break;
        }
    }
    
    return highlightedSpans;    
}

function clearCurrentSearchMatch()
{
    // clear prior highlighting
    spanName = "highlight" + currentlyHighlightedHit;
    highlightedSpans = getSpansByName(spanName);
    if (highlightedSpans && highlightedSpans.length) {
        for (i = 0; i < highlightedSpans.length; i++) {
            if (highlightedSpans[i]) {
                highlightedSpans[i].setAttribute("class", "highlighted");
            }           
        }
    }
}

function setCurrentSearchMatchIfVisible(hitNum){
    currentlyHighlightedHit = hitNum;
    var spanName = "highlight" + currentlyHighlightedHit;
    var highlightedSpans = getSpansByName(spanName);
    if (highlightedSpans && highlightedSpans.length) {
        for (i = 0; i < highlightedSpans.length; i++) {
            if (!isVisibleElement(highlightedSpans[i])) {
                highlightedSpans = null;
                break;
            }
        }
    }
    return highlightedSpans;
}

// this takes in an option integer 'hitNum'. If not specified, it scrolls
// to the next hit
function scrollToNextHit(hitNum)
{
    var i = 0;
    var found = false;
    var spanName = "";
    var highlightedSpans;
    var searchBox = findElement('searchTxtBox');
    
    clearCurrentSearchMatch();
    
    if (hitNum) {
        // if a number is provided, use it
        highlightedSpans = setCurrentSearchMatchIfVisible(hitNum);
    } else {        
        // start working on next element to highlight
        highlightedSpans = getNextVisible();
    }    
    
    // we found the current 
    if (highlightedSpans && highlightedSpans.length > 0) {
        highlightedSpans[0].scrollIntoView();
        for (i = 0; i < highlightedSpans.length; i++) {
            highlightedSpans[i].setAttribute("class", "highlightedCurrent");
        }
        searchBox.setAttribute("class", "search");
        
    // if highlightedSpans is invalid, then we did not find any valid, visible subsequent matches
    // wrap to beginning or indicate no matches
    } else {
        // Element not found. Scroll to first visible element        
        currentlyHighlightedHit = 0;
        var highlightedSpans = getNextVisible(currentlyHighlightedHit);
        if (highlightedSpans && highlightedSpans.length > 0) {
            highlightedSpans[0].scrollIntoView();
            for (i = 0; i < highlightedSpans.length; i++) {
                highlightedSpans[i].setAttribute("class", "highlightedCurrent");
            }
            searchBox.setAttribute("class", "search");
        } else {
            // there aren't any matches
            searchBox.setAttribute("class", "failedSearch");
        }
    }
}

// find search box
function findElement(element)
{
    var i = 0;
    for (i = 0; i < top.frames.length; i++) {
        if (top.frames[i].name === "rtwreport_contents_frame" || top.frames[i].name === "rtwreport_document_frame") {
            var elem = top.frames[i].document.getElementById(element);
            if (elem) { 
                return elem;
            }
        }
    }
}

// Restore search term once form is submitted
function initSearchBox(strInitValue)
{
    var txtBox = findElement('searchTxtBox');
    if (txtBox) {
        txtBox.value = strInitValue;
    }
}

// Sets focus back on to the text box
function setFocusOnTxtBox()
{
    var txtBox = findElement('searchTxtBox');
    if (txtBox) {
        txtBox.focus();
        txtBox.select();
    }
    return txtBox;
}

var previousSearchString;
var totalHits;
function findInAllDocs(searchStringFromSubmitBox, caseSensitive)
{
    if (previousSearchString != searchStringFromSubmitBox) {
        // If the search string has changed or a new page has been loaded, do a new search
        var hitCount = 1;
        var i = 0;        
        var success = false;
        previousSearchString = searchStringFromSubmitBox;
               
        // start by removing traceinfo highlighting
        rtwRemoveHighlighting();
        
        // remove all previous search highlighting
        removeHighlightInAllDocs();

        // 1. Iterate through all frames in window and search
        for (i = 0; i < top.frames.length; i++) {
            var currentDoc = top.frames[i].document;    
            
            // if we have no search term, restore
            if (searchStringFromSubmitBox !== "") {
                // search and highlight in all frames
                var srchResult = findInDoc(searchStringFromSubmitBox, currentDoc, caseSensitive, hitCount);
                hitCount = srchResult[1];
                totalHits = srchResult[1];
            }
        }

        // 2. Restore search term once form is submitted and text highlighted        
        if (searchStringFromSubmitBox != "") {
            strInitValue = searchStringFromSubmitBox;
        }    
        initSearchBox(strInitValue);

        // 3. Scroll to the first hit encountered
        scrollToNextHit(1);
        
        // 4. Set focus back to text box and select text
        var txtBox = setFocusOnTxtBox();
        if (txtBox) {
            txtBox.select();
        }
                
    } else {
        // If the search string is the same, then scroll to the next 
        // hit if the hit is valid. Else wrap back.        
        scrollToNextHit();
    }
    return false;
}

// if the search box is empty, clear highlighting
function clearIfEmpty()
{
    txtBox = findElement('searchTxtBox');
    if (txtBox.value == "") {
        txtBox.setAttribute("class", "search");
        removeHighlightInAllDocs();
        previousSearchString="";
        setFocusOnTxtBox();
    }
}

function keyPressSwitchyard(keyPressEvent)
{ 
    var kc;
    keyPressEvent = (keyPressEvent == null ? window.keyPressEvent : keyPressEvent);

    // typically IE does not support this
    if (!keyPressEvent || (typeof keyPressEvent == "undefined")) {
        return;
    }
    
    if (keyPressEvent.keyCode) {    
        kc=keyPressEvent.keyCode;
    } else if (keyPressEvent.which) {
        kc=keyPressEvent.which;   
    } else {
        return;
    }

    // we do not care about the browser find appearing. If it does appear, then 
    // we are running an external browser and that is okay.
    
    // handle Ctrl-Key combinations
    if (keyPressEvent.ctrlKey) {
        switch (kc) {
            case 70: // Ctrl-F
            { 
              setFocusOnTxtBox();
              break;
            }
            
            default: break;
        }
    } 
}

function installDocumentKeyPressHandler()
{
    var i = 0;
    for (i = 0; i < top.frames.length; i++) {
        var currentDoc = top.frames[i].document;    
        currentDoc.onkeydown = keyPressSwitchyard;
    }    
    top.document.onkeydown = keyPressSwitchyard;
    
    // This also clears search related highlighting
    removeHighlightInAllDocs();
    currentlyHighlightedHit = 0;
    if (previousSearchString) initSearchBox(previousSearchString);
    previousSearchString = "";
}

// This function is a onresize callback for the rtwreport_contents_frame
// It is used to dynamically resize the searchbox based on the size of the frame.
function setWidthDynamic(frameID, elementID, extraSpace, minSize)
{
    var frame = document.getElementById(frameID);
    
    // sanity check input args
    if (frame && extraSpace > 0 && minSize > 0) {
        var frameWidth = frame.scrollWidth;
        var newSize = extraSpace + minSize + 40; // 40 is the extra whitespace
        var element = findElement(elementID);
        if (element)
        {
            if (frameWidth < newSize) {
                element.style.width = minSize;
            } else {
                element.style.width = frameWidth - extraSpace - 40;
            }
        }
    }
}
