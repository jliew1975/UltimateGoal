<?php

if($_POST["Send"]) {
    $recipient="john.liew@eborgrobotics.org";
    $subject="Contact from Website";
    $sender=$_POST["firstName"] . " " . $_POST["lastName"];
    $senderEmail=$_POST["userEmail"];
    $message=$_POST["message"];

    $mailBody="Name: $sender\nEmail: $senderEmail\n\n$message";

    mail($recipient, $subject, $mailBody, "From: $sender <$senderEmail>");

    $thankYou="<p>Thank you! Your message has been sent.</p>";
}

?>
<!DOCTYPE html>
<html lang="en">

<head><meta http-equiv="Content-Type" content="text/html; charset=euc-jp">
    
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="X-UA-Compatible" content="IE=edge" />
    <!-- The above 3 meta tags *must* come first in the head; any other head content must come *after* these tags -->
    <!-- Title -->
    <title>eBorg Robotics Inc</title>

    <!-- ================================================================
        ***Favicon***
    ================================================================= -->

    <link rel="shortcut icon" type="image/png" href="images/fevicon.png">

    <!-- ================================================================
        ***CSS File***
    ================================================================= -->
    <!-- ================= *** Google Font-Poppins *** ======================= -->
    <link href="https://fonts.googleapis.com/css?family=Roboto:100,100i,300,300i,400,400i,500,500i,700,700i,900,900i" rel="stylesheet">

    <!-- ================= *** Animate CSS *** ======================= -->
    <link href="css/animate.min.css" rel="stylesheet" type="text/css">

    <!-- ================= *** Bootstrap CSS *** ===================== -->
    <link href="css/bootstrap.min.css" rel="stylesheet" type="text/css">

    <!-- ================= *** Font-awesome CSS *** ================== -->
    <link href="css/hover-min.css" rel="stylesheet" type="text/css">

    <!-- ================= *** icofont CSS *** ======================= -->
    <link href="css/icofont.css" rel="stylesheet" type="text/css">

    <!-- ================= *** progressbar CSS *** ======================= -->
    <link href="css/jQuery-plugin-progressbar.css" rel="stylesheet" type="text/css">

    <!-- ================= *** Owl Carousel CSS *** ================== -->
    <link href="css/owl.carousel.css" rel="stylesheet" type="text/css">

    <!-- ================= *** Main CSS *** ========================== -->

    <link href="css/custom/style.css" rel="stylesheet" type="text/css">

    <!-- HTML5 shim and Respond.js for IE8 support of HTML5 elements and media queries -->
    <!-- WARNING: Respond.js doesn't work if you view the page via file:// -->
    <!--[if lt IE 9]>
      <script src="https://oss.maxcdn.com/html5shiv/3.7.2/html5shiv.min.js"></script>
      <script src="https://oss.maxcdn.com/respond/1.4.2/respond.min.js"></script>
    <![endif]-->
</head>

<body class="home">
    <div id="preloader">
        <div class="preloader_spinner"></div>
    </div>
    <!-- preloader end -->
<!-- ==========================================================
    1.*Header_area start
============================================================ -->
    <header>
        <!-- Header-Top End -->
        <div class="romana_header_bottom">
            <div class="container">
                <div class="row">
                    <div class="col-sm-2 col-xs-6">
                        <div class="romana_logo">
                            <a href="index.html"><img src="images/logo.png" alt=""></a>
                        </div>
                    </div>
                    <!-- column End -->
                    <div class="col-sm-10">
                        <div class="navbar-header">
                            <button type="button" class="navbar-toggle collapsed" data-toggle="collapse" data-target="#navbar" aria-expanded="false" aria-controls="navbar">
                                <span class="sr-only"></span>
                                <i class="icofont icofont-navigation-menu"></i>
                            </button>
                        </div>
                        <div class="navbar-header">
                            <button type="button" class="navbar-toggle collapsed" data-toggle="collapse" data-target="#navbar" aria-expanded="false" aria-controls="navbar">
                                <span class="sr-only"></span>
                                <i class="icofont icofont-navigation-menu"></i>
                            </button>
                        </div>
                        <nav id="navbar" class="navbar-collapse collapse">
                            <ul class="nav navbar-nav">
                                <li><a href="index.html">Home</a></li>
                                <li><a href="#">about</a>
                                    <ul class="dropdown-menu">
                                        <li><a href="about.html">Our Mission</a></li>
                                        <li><a href="team.html">Our Team</a></li>
                                        <li><a href="https://www.firstinspires.org/" target="_blank">About FIRST</a></li>
                                    </ul>
                                </li>
                                <li><a href="achievements.html">Achievements</a></li>
                                <li><a href="#">events</a>
                                    <ul class="dropdown-menu">
                                        <li><a href="outreaches.html">Outreaches</a></li>
                                        <li><a href="upcoming_outreaches.html">Upcoming Events</a></li>
                                    </ul>
                                </li>
                                <li><a href="#">News Feed</a>
                                    <ul class="dropdown-menu">
                                        <li><a href="news.html">Robotics News</a></li>
                                    </ul>
                                </li>
                                <li><a href="sponsor.html">Our Sponsors</a></li>
                                <li class="active"><a href="contact.html">Contact</a></li>
                            </ul>
                            <!-- mega-menu End -->
                        </nav>
                        <!-- nav End -->
                        <ul class="cartSearch">
                            <!--
                            <li class="search"><a href="#"><i class="icofont icofont-search"></i></a></li>
                            -->
                            <li class="donation"><a href="https://www.gofundme.com/eborg-robotics-inc" class="hvr-box-shadow-outset" target="_blank">Donate Now</a></li>
                        </ul>
                        <!-- cartSearch End -->
                        <form action="#" class="searchForm">
                            <i class="icofont icofont-close"></i>
                            <input type="search" placeholder="Search now">
                            <input type="submit" value="&#xeded;">
                        </form>
                        <!-- searchForm End -->
                    </div>
                    <!-- column End -->
                </div>
                <!-- row End -->
            </div>
            <!-- container End -->
        </div>
        <!-- Header Bottom End -->
    </header>
    <!-- Header End -->
<!-- ==========================================================
2.*Hero_area start
============================================================ -->
    <div class="romana_allPage_area">
        <div class="container">
            <div class="row">
                <div class="col-xs-12">
                    <div class="romana_allPage_text text-center">
                        <h1>contact</h1>
                        <ol class="breadcrumb">
                            <li><a href="index.html">Home</a><span></span></li>
                            <li><a href="#">Contact</a></li>
                        </ol>
                    </div>
                </div>
                <!-- column End -->
            </div>
            <!-- row End -->
        </div>
        <!-- container End -->
    </div>
<!-- ==========================================================
3.*romana_contact_form_area start
============================================================ -->
    <div class="romana_contact_form_area romana_section_padding">
        <div class="container">
            <div class="row ">
                <div class="col-md-6 col-sm-12">
                    <div id="g_Map"></div>
                </div>
                <!-- column End -->
                <div class="col-md-6 col-sm-12">
                    <div class="romana_contact_form romana_common_form">
                        <form action="contact.php" method="POST">
                            <div class="row">
                                <div class="col-sm-6">
                                    <div class="field">                                        
                                        <input id="FistName" type="text" name="firstName" placeholder="First Name" />
                                    </div>
                                    <div class="field">
                                        <label class="placeholder" for="userEmail">
                                            Email Address
                                        </label>
                                        <input id="userEmail" type="text" name="userEmail" />
                                    </div>
                                </div>
                                <!-- column End -->
                                <div class="col-sm-6">
                                    <div class="field">
                                        <label class="placeholder" for="LastName">
                                            Last name
                                        </label>
                                        <input id="LastName" type="text" name="lastName" />
                                    </div>
                                    <div class="field">
                                        <label class="placeholder" for="userPhone">
                                            Phone Number
                                        </label>
                                        <input id="userPhone" type="text" name="userPhone" />
                                    </div>
                                </div>
                                <!-- column End -->
                            </div>
                            <!-- row End -->
                            <div class="row">
                                <div class="col-sm-12">
                                    <div class="field text-field">
                                        <label class="placeholder">
                                            Message
                                        </label>
                                        <textarea cols="30" rows="10" name="message"></textarea>
                                    </div>
                                    <div class="romana_submit_btn">
                                        <input type="submit" value="Send">
                                    </div>
                                </div>
                                <!-- column End -->
                            </div>
                            <!-- row End -->
                        </form>
                    </div>
                    <!-- romana_registration form End -->
                </div>
                <!-- column End -->
            </div>
            <!-- row End -->
            <div class="romana_all_contact">
                <div class="row">
                    <!--
                    <div class="col-sm-4">
                        <div class="romana_single_contact clearfix">
                            <div class="romana_contact_icon">
                                <a href="#"><i class="icofont icofont-ui-call"></i></a>
                            </div>
                            <div class="romana_contact_info text-center">
                                <p>+(00)111-222-333</p>
                                <p>+(00)44-556-6678</p>
                            </div>
                        </div>
                    </div>
                    -->
                    <!-- column End -->
                    <div class="col-sm-2"></div>
                    <div class="col-sm-4">
                        <div class="romana_single_contact clearfix">
                            <div class="romana_contact_icon">
                                <a href="#"><i class="icofont icofont-ui-message"></i></a>
                            </div>
                            <div class="romana_contact_info text-center">
                                <p>contact@eborgrobotics.org</p>
                                <p>&nbsp;</p>
                            </div>
                        </div>
                    </div>
                    <!-- column End -->
                    <div class="col-sm-4">
                        <div class="romana_single_contact clearfix">
                            <div class="romana_contact_icon">
                                <a href="#"><i class="icofont icofont-social-google-map"></i></a>
                            </div>
                            <div class="romana_contact_info text-center">
                                <p>50 Route 10 West</p>
                                <p>East Hanover, NJ 07936</p>
                            </div>
                        </div>
                    </div>
                    <div class="col-sm-2"></div>
                    <!-- column End -->
                </div>
                <!-- row End -->
            </div>
        </div>
        <!-- container End -->
    </div>
<!-- ==================================================
4.*Footer_area start
=================================================== -->
<footer class="romana_footer_area romana_section_padding">
    <div class="romana_footer_top">
        <div class="container">
            <div class="row">
                <div class="col-md-3 col-sm-6">
                    <div class="footer_left_text">
                        <div class="footer_logo">
                            <a href="#"><img src="images/footer_logo.png" alt="logo"></a>
                        </div>
                        <p>Help us spread the passion of Science, Technology, Engineering, and Math (STEM). Help Team eBorg (Team #12538) enter their first robotic competition at FIRST Tech Challenge.</p>
                    </div>
                </div>
                <!-- column End -->
                <div class="col-md-3 col-sm-6">
                    <div class="footer_menu">
                        <h2>FIRST&reg; Tech Challenge</h2>
                        <ul>
                            <li><i class="icofont icofont-thin-right"></i><a href="http://www.firstinspires.org/" target="_blank">About FIRST&reg;</a></li>
                            <li><i class="icofont icofont-thin-right"></i><a href="https://www.firstinspires.org/robotics/ftc" target="_blank">About FTC</a></li>
                        </ul>
                    </div>
                </div>
                <!-- column End -->
                <div class="col-md-3 col-sm-6">
                    <div class="footer_recent_news">
                        <h2>Recent Outreaches</h2>
                        <div class="romana_single_footer_news">
                            <div class="romana_single_footer_news_img">
                                <a href="outreaches_ymca.html"><img src="images/outreaches_ymca/ymca_2.jpg" alt="" style="width: 50px; height: 50px;"></a>
                            </div>
                            <div class="romana_single_footer_news_text">
                                <a href="outreaches_ymca.html"><h3>Livingston YMCA Outreach</h3></a>
                                <p>May 21, 2017</p>
                            </div>
                        </div>
                        <div class="romana_single_footer_news">
                            <div class="romana_single_footer_news_img">
                                <a href="outreaches_millburn.html"><img src="images/outreaches_millburn/millburn_8.jpg" alt="" style="width: 50px; height: 50px;"></a>
                            </div>
                            <div class="romana_single_footer_news_text">
                                <a href="outreaches_millburn.html"><h3>Millburn High School</h3></a>
                                <p>July 9, 2017</p>
                            </div>
                        </div>
                        <div class="romana_single_footer_news">
                            <div class="romana_single_footer_news_img">
                                <a href="outreaches_india.html"><img src="images/outreaches_india/india_1.jpg" alt="" style="width: 50px; height: 50px;"></a>
                            </div>
                            <div class="romana_single_footer_news_text">
                                <a href="outreaches_india.html"><h3>India Outreach</h3></a>
                                <p>July 15, 2017</p>
                            </div>
                        </div>
                    </div>
                </div>
                <!-- column End -->
                <div class="col-md-2 col-md-offset-1 col-sm-6">
                    <div class="footer_contact">
                        <h2>Contact Us</h2>
                        <div class="romana_single_contact">
                            <p>Email us <a href="mailto:contact@eborgrobotics.org"><span>contact@eborgrobotics.org</span></a></p>
                        </div>
                        <div class="romana_single_contact">
                            <address>
                            Address
                            <span>
                                eBorg Robotics Inc.
                                50 Route 10 West<br>
                                East Hanover, New Jersey 07936
                            </span>
                        </address>
                        </div>
                    </div>
                </div>
                <!-- column End -->
            </div>
        </div>
    </div>
    <!-- footer_top End -->
    <div class="romana_footer_bottom">
        <div class="container">
            <div class="row">
                <div class="col-xs-12">
                    <div class="romana_footer_bottom_text text-center">
                        <p>&copy;2017 eBorg Robotics Inc. All right reserved</p>
                    </div>
                </div>
                <!-- column End -->
            </div>
            <!-- row End -->
        </div>
        <!-- container End -->
    </div>
    <!-- footer_bottom End -->
</footer>
 <!-- ======================================================
    ***Js Files***
=========================================================== -->
    <!-- ================= Main Js ==================== -->
    <script src="js/jquery-3.1.0.min.js"></script>
    <!-- Include all compiled plugins (below), or include individual files as needed -->
    <script src="https://maps.googleapis.com/maps/api/js?key=AIzaSyD7CQl6fRhagGok6CzFGOOPne2X1u1spoA"></script>
    <script type="text/javascript">
        var map;
        var latlng = new google.maps.LatLng(40.813741, -74.394547);
        var stylez = [
            {
                featureType: "all",
                elementType: "all",
                stylers: [
                    {
                        saturation: -100
                    } // <-- THIS
                  ]
                }
            ];
        var mapOptions = {
            zoom: 13,
            center: latlng,
            scrollwheel: false,
            scaleControl: false,
            disableDefaultUI: true,
            mapTypeControlOptions: {
                mapTypeIds: [google.maps.MapTypeId.ROADMAP, 'gMap']
            }
        };
        map = new google.maps.Map(document.getElementById("g_Map"), mapOptions);
        var geocoder_map = new google.maps.Geocoder();
        var address = '50 Route 10 West, East Hanover, NJ 07936';
        geocoder_map.geocode({
            'address': address
        }, function (results, status) {
            if (status == google.maps.GeocoderStatus.OK) {
                map.setCenter(results[0].geometry.location);

                var marker = new google.maps.Marker({
                    map: map,
                    icon: 'images/map.png',
                    position: map.getCenter()
                });

                var contentString = 'Rome';
                var infowindow = new google.maps.InfoWindow({
                    content: contentString
                });

            } else {
                alert("Geocode was not successful for the following reason: " + status);
            }
        });
        var mapType = new google.maps.StyledMapType(stylez, {
            name: "Grayscale"
        });
        map.mapTypes.set('gMap', mapType);
        map.setMapTypeId('gMap');
    </script>
    <!-- ================= Bootstrap min Js =========== -->
    <script src="js/bootstrap.min.js"></script>

    <!-- ================= owl carousel min Js ======== -->
    <script src="js/owl.carousel.min.js"></script>

    <!-- ================= progressbar Js ======== -->
    <script src="js/jQuery-plugin-progressbar.js"></script>

    <!-- ================= Active Js ================== -->
    <script src="js/active.js"></script>
</body>

</html>
