<?php

/**
 * Facedetect
 *
 * Copyright (c) 2010, Robert Eisele (robert@xarg.org)
 * Dual licensed under the MIT or GPL Version 2 licenses.
 **/

// Get Contents from flickr
$cont_= file_get_contents('http://api.flickr.com/services/feeds/photos_public.gne?tags=face&format=rss_100');

// Parse the feed
preg_match_all('|&lt;img.*src=&quot;(.*jpg)&quot;.*/&gt;|', $cont_, $out);

foreach($out[1] as $o) {

	$path = 'faces/'.crc32($o).'.jpg';
	// Save Image
	file_put_contents($path, file_get_contents($o));

	// Find faces
	$faces = face_detect($path, 'haarcascades/haarcascade_frontalface_alt.xml');

	// Draw
	foreach($faces as $x) {
		exec('convert '.$path.' -fill transparent -stroke red -draw "circle '.($x['x']+$x['w']/2).','.($x['y']+$x['h']/2).' '.($x['x']+$x['w']).','.($x['y']+$x['h']).'" '.$path);
	}
}

?>